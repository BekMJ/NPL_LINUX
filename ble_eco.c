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

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_ECO)
#include "ble_eco.h"

#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"
#include "ble_conn_state.h"

#include "nrf_log.h"


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_eco       Environmental Sensing structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_eco_t * p_eco, ble_evt_t const * p_ble_evt)
{

    uint32_t                      err_code;
    ble_gatts_value_t             gatts_value;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (    (p_evt_write->handle == p_eco->concentration_handles.cccd_handle)
        &&  (p_evt_write->len == 2))
    {
        if (p_eco->evt_handler == NULL)
        {
            NRF_LOG_INFO("Event Handler NULL");
            return;
        }

        ble_eco_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            evt.evt_type = BLE_ECO_EVT_NOTIFICATION_ENABLED;
        }
        else
        {
            evt.evt_type = BLE_ECO_EVT_NOTIFICATION_DISABLED;
        }
        evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;

        // CCCD written, call application event handler.
        p_eco->evt_handler(p_eco, &evt);
        NRF_LOG_INFO("CCCD");
    }
    else if (    (p_evt_write->handle == p_eco->test_handles.cccd_handle)
        &&  (p_evt_write->len == 2))
    {
        ble_eco_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            evt.evt_type = BLE_ECO_EVT_NOTIFICATION_ENABLED;
            // Initialize value struct.
            memset(&gatts_value, 0, sizeof(gatts_value));

            gatts_value.len     = 2*sizeof(uint8_t);
            gatts_value.offset  = 0;
            gatts_value.p_value = p_eco->test_value;

            // Update database.
            err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                              p_eco->test_handles.value_handle,
                                              &gatts_value);
            
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_eco->test_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();
            
            err_code = sd_ble_gatts_hvx(conn_handles.conn_handles[0], &hvx_params);

        }
        else
        {
            evt.evt_type = BLE_ECO_EVT_NOTIFICATION_DISABLED;
        }

    }
    else if(p_evt_write->handle == p_eco->command_handles.value_handle )
    {
      
      uint8_t status_value = p_evt_write->data[0];
      uint8_t opcode_value = p_evt_write->data[1];
      uint8_t register_value[2] = {status_value, opcode_value};
      NRF_LOG_INFO("status_value: %d", (int)status_value);
      if (status_value == 2)
      {
        if (opcode_value == 0){
          register_value[0] = p_eco->device_info[0];
        }
        else if (opcode_value == 1)
        {
          register_value[0] = p_eco->device_info[1] >> 2;
          register_value[1] = p_eco->device_info[1] >> 10;
        }
        else if (opcode_value == 2)
        {
          register_value[0] = p_eco->device_info[3] >> 0;
          register_value[1] = p_eco->device_info[3] >> 8;
        }
        else if (opcode_value == 1)
        {
          register_value[0] = p_eco->device_info[3] >> 16;
          register_value[1] = p_eco->device_info[3] >> 24;
        }
      }
      *p_eco->status_value = register_value[0];
      *p_eco->test_value = p_eco->device_info[1];

      // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = 2*sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = register_value;

        // Update database.
        err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                          p_eco->status_handles.value_handle,
                                          &gatts_value);

      memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = 2*sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = register_value;
        err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                          p_eco->test_handles.value_handle,
                                          &gatts_value);
    }
}

void ble_eco_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_eco_t * p_eco = (ble_eco_t *)p_context;
//    NRF_LOG_INFO("Event");

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            //NRF_LOG_INFO("Subscribe");
            on_write(p_eco, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            //NRF_LOG_INFO("Data");
            //on_write(p_eco, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Characteristic.
 *
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[in]   char_len       Length of initial value. This will also be the maximum value.
 * @param[in]   rd_sec         Security requirement for reading characteristic value.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_eco_init(ble_eco_t * p_eco, ble_eco_init_t const * p_eco_init)
{
    uint32_t                err_code;
    uint16_t                initial_value = 0;
    ble_uuid_t              ble_uuid;
    ble_add_char_params_t   add_char_params;

    // Initialize service structure
    p_eco->evt_handler               = p_eco_init->evt_handler;
    p_eco->concentration_value       = p_eco_init->concentration_value;
    p_eco->temperature_value         = p_eco_init->temperature_value;
    p_eco->humidity_value            = p_eco_init->humidity_value;
    p_eco->pressure_value            = p_eco_init->pressure_value;
    p_eco->status_value              = p_eco_init->status_value;
    p_eco->test_value                = p_eco_init->test_value;
    p_eco->device_info               = p_eco_init->device_info;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ECO_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_eco->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics
    // Characteristic 1: Carbon Monoxide Concentration (parts per million)
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                      = BLE_UUID_CO_CONCENTRATION;
//    add_char_params.uuid                      = BLE_UUID_TEMPERATURE;
    add_char_params.max_len                   = 2*sizeof(uint8_t);
    add_char_params.init_len                  = 2*sizeof(uint8_t);
    add_char_params.p_init_value              = &initial_value;
    add_char_params.cccd_write_access         = SEC_OPEN;
    add_char_params.char_props.indicate       = 0;
    add_char_params.char_props.notify         = 1;
    add_char_params.char_props.read           = 1;
    add_char_params.read_access               = SEC_OPEN;
    add_char_params.char_props.write          = 0;
    add_char_params.char_props.write_wo_resp  = 0;
    add_char_params.write_access              = SEC_OPEN;
    
    err_code = characteristic_add(p_eco->service_handle, &add_char_params, &(p_eco->concentration_handles));
    VERIFY_SUCCESS(err_code);
    
    // Characteristic 2: Status Flags
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                      = BLE_UUID_STATUS_BYTE;
    add_char_params.max_len                   = 2*sizeof(uint8_t);
    add_char_params.init_len                  = 2*sizeof(uint8_t);
    add_char_params.p_init_value              = &initial_value;
    add_char_params.cccd_write_access         = SEC_OPEN;
    add_char_params.char_props.notify         = 1;
    add_char_params.char_props.read           = 1;
    add_char_params.read_access               = SEC_OPEN;
    add_char_params.char_props.write          = 0;
    add_char_params.char_props.write_wo_resp  = 0;
    add_char_params.write_access              = SEC_OPEN;
    
    err_code = characteristic_add(p_eco->service_handle, &add_char_params, &(p_eco->status_handles));
    VERIFY_SUCCESS(err_code); 
    
    // Characteristic 3: Command register
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                      = BLE_UUID_COMMAND_BYTE;
    add_char_params.max_len                   = 6*sizeof(uint8_t);
    add_char_params.init_len                  = 2*sizeof(uint8_t);
    add_char_params.p_init_value              = &initial_value;
    add_char_params.char_props.notify         = 0;
    add_char_params.char_props.read           = 0;
    add_char_params.read_access               = SEC_OPEN;
    add_char_params.char_props.write          = 1;
    add_char_params.char_props.write_wo_resp  = 1;
    add_char_params.write_access              = SEC_OPEN;
    
    err_code = characteristic_add(p_eco->service_handle, &add_char_params, &p_eco->command_handles);
    VERIFY_SUCCESS(err_code); 
    
    // Characteristic 4: Test result
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                      = BLE_UUID_TEST_BYTE;
    add_char_params.max_len                   = 2*sizeof(uint8_t);
    add_char_params.init_len                  = 2*sizeof(uint8_t);
    add_char_params.p_init_value              = &initial_value;
    add_char_params.cccd_write_access         = SEC_OPEN;
    add_char_params.char_props.notify         = 1;
    add_char_params.char_props.read           = 1;
    add_char_params.read_access               = SEC_OPEN;
    add_char_params.char_props.write          = 0;
    add_char_params.char_props.write_wo_resp  = 0;
    add_char_params.write_access              = SEC_OPEN;
    
    err_code = characteristic_add(p_eco->service_handle, &add_char_params, &(p_eco->test_handles));
    VERIFY_SUCCESS(err_code);
   
    // Characteristic 5: Temperature
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                      = BLE_UUID_TEMPERATURE;
    add_char_params.max_len                   = 2*sizeof(uint8_t);
    add_char_params.init_len                  = 2*sizeof(uint8_t);
    add_char_params.p_init_value              = &initial_value;
    add_char_params.cccd_write_access         = SEC_OPEN;
    add_char_params.char_props.indicate       = 0;
    add_char_params.char_props.notify         = 1;
    add_char_params.char_props.read           = 1;
    add_char_params.read_access               = SEC_OPEN;
    add_char_params.char_props.write          = 0;
    add_char_params.char_props.write_wo_resp  = 0;
    add_char_params.write_access              = SEC_OPEN;
    
    err_code = characteristic_add(p_eco->service_handle, &add_char_params, &(p_eco->temperature_handles));
    
    // Characteristic 6: Humidity
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                      = BLE_UUID_HUMIDITY;
    add_char_params.max_len                   = 2*sizeof(uint8_t);
    add_char_params.init_len                  = 2*sizeof(uint8_t);
    add_char_params.p_init_value              = &initial_value;
    add_char_params.cccd_write_access         = SEC_OPEN;
    add_char_params.char_props.indicate       = 0;
    add_char_params.char_props.notify         = 1;
    add_char_params.char_props.read           = 1;
    add_char_params.read_access               = SEC_OPEN;
    add_char_params.char_props.write          = 0;
    add_char_params.char_props.write_wo_resp  = 0;
    add_char_params.write_access              = SEC_OPEN;
    
    err_code = characteristic_add(p_eco->service_handle, &add_char_params, &(p_eco->humidity_handles));
    
    // Characteristic 7: Pressure
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                      = BLE_UUID_PRESSURE;
    add_char_params.max_len                   = 4*sizeof(uint8_t);
    add_char_params.init_len                  = 4*sizeof(uint8_t);
    add_char_params.p_init_value              = &initial_value;
    add_char_params.cccd_write_access         = SEC_OPEN;
    add_char_params.char_props.indicate       = 0;
    add_char_params.char_props.notify         = 1;
    add_char_params.char_props.read           = 1;
    add_char_params.read_access               = SEC_OPEN;
    add_char_params.char_props.write          = 0;
    add_char_params.char_props.write_wo_resp  = 0;
    add_char_params.write_access              = SEC_OPEN;
    
    err_code = characteristic_add(p_eco->service_handle, &add_char_params, &(p_eco->pressure_handles));
    
    
    VERIFY_SUCCESS(err_code);
    
    return NRF_SUCCESS;
}

/**@brief Function for sending notifications with the Battery Level characteristic.
 *
 * @param[in]   p_hvx_params Pointer to structure with notification data.
 * @param[in]   conn_handle  Connection handle.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t notification_send(ble_gatts_hvx_params_t * const p_hvx_params,
                                            uint16_t                       conn_handle)
{
    ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);
    
    return err_code;
}

ret_code_t ble_eco_concentration_update(ble_eco_t * p_eco,
                                        uint16_t    conn_handle)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;

    if (*p_eco->concentration_value != p_eco->concentration_last)
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = 2*sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = p_eco->concentration_value;

        // Update database.
        err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                          p_eco->concentration_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            p_eco->concentration_last = *p_eco->concentration_value;
        }
        else
        {
            return err_code;
        }

        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_eco->concentration_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

            if (conn_handle == BLE_CONN_HANDLE_ALL)
            {
                ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

                // Try sending notifications to all valid connection handles.
                for (uint32_t i = 0; i < conn_handles.len; i++)
                {
                    if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
                    {
                        if (err_code == NRF_SUCCESS)
                        {
                            err_code = notification_send(&hvx_params,
                                                                 conn_handles.conn_handles[i]);
                        }
                        else
                        {
                            // Preserve the first non-zero error code
                            UNUSED_RETURN_VALUE(notification_send(&hvx_params,
                                                                          conn_handles.conn_handles[i]));
                        }
                    }
                }
            }
            else
            {
                err_code = notification_send(&hvx_params, conn_handle);
            }
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }

    return err_code;
}



ret_code_t ble_eco_temperature_update(ble_eco_t * p_eco,  uint16_t conn_handle)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;
    
    NRF_LOG_INFO("*p_eco->temperature_value:   %d deg C\n", *p_eco->temperature_value);
    
    if ( *p_eco->temperature_value!= p_eco->temperature_last)
    {
	    // Initialize value struct.
	    memset(&gatts_value, 0, sizeof(gatts_value));

	    gatts_value.len     = 2*sizeof(uint8_t);
	    gatts_value.offset  = 0;
	    gatts_value.p_value = p_eco->temperature_value;

	    // Update database.
	    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
		                                  p_eco->temperature_handles.value_handle,
		                                  &gatts_value);
	    
	    if (err_code == NRF_SUCCESS)
	    {
	    	p_eco->temperature_last = *p_eco->temperature_value;
	    }
	    else
	    {
	    	return err_code;
	    }
		            
	    ble_gatts_hvx_params_t hvx_params;
	    memset(&hvx_params, 0, sizeof(hvx_params));

	    hvx_params.handle = p_eco->temperature_handles.value_handle;
	    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	    hvx_params.offset = gatts_value.offset;
	    hvx_params.p_len  = &gatts_value.len;
	    hvx_params.p_data = gatts_value.p_value;
	    
	    NRF_LOG_INFO("hvx_params.p_data: %d", *hvx_params.p_data);
	    
	//    NRF_LOG_INFO("conn_handle: %d", (int)conn_handle);
	    if (conn_handle == BLE_CONN_HANDLE_ALL)
	    {
	    	ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();
	    	
	    	// Try sending notifications to all valid connection handles.
		for (uint32_t i = 0; i < conn_handles.len; i++)
		{
			if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
			{
				if (err_code == NRF_SUCCESS)
		                {
		                    err_code = notification_send(&hvx_params,
		                                                         conn_handles.conn_handles[i]);
		                }
		                else
		                {
		                    // Preserve the first non-zero error code
		                    UNUSED_RETURN_VALUE(notification_send(&hvx_params,
		                                                                  conn_handles.conn_handles[i]));
		                }
		       }
		 }
	    }   
    }
    
    
    return err_code;
    
}

ret_code_t ble_eco_humidity_update(ble_eco_t * p_eco,  uint16_t conn_handle)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;
    
    NRF_LOG_INFO("*p_eco->humidity_value:   %d deg C\n", *p_eco->humidity_value);
    
    if ( *p_eco->humidity_value!= p_eco->humidity_last)
    {
	    // Initialize value struct.
	    memset(&gatts_value, 0, sizeof(gatts_value));

	    gatts_value.len     = 2*sizeof(uint8_t);
	    gatts_value.offset  = 0;
	    gatts_value.p_value = p_eco->humidity_value;

	    // Update database.
	    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
		                                  p_eco->humidity_handles.value_handle,
		                                  &gatts_value);
	    
	    if (err_code == NRF_SUCCESS)
	    {
	    	p_eco->humidity_last = *p_eco->humidity_value;
	    }
	    else
	    {
	    	return err_code;
	    }
		            
	    ble_gatts_hvx_params_t hvx_params;
	    memset(&hvx_params, 0, sizeof(hvx_params));

	    hvx_params.handle = p_eco->humidity_handles.value_handle;
	    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	    hvx_params.offset = gatts_value.offset;
	    hvx_params.p_len  = &gatts_value.len;
	    hvx_params.p_data = gatts_value.p_value;
	    
	    NRF_LOG_INFO("hvx_params.p_data: %d", *hvx_params.p_data);
	    
	//    NRF_LOG_INFO("conn_handle: %d", (int)conn_handle);
	    if (conn_handle == BLE_CONN_HANDLE_ALL)
	    {
	    	ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();
	    	
	    	// Try sending notifications to all valid connection handles.
		for (uint32_t i = 0; i < conn_handles.len; i++)
		{
			if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
			{
				if (err_code == NRF_SUCCESS)
		                {
		                    err_code = notification_send(&hvx_params,
		                                                         conn_handles.conn_handles[i]);
		                }
		                else
		                {
		                    // Preserve the first non-zero error code
		                    UNUSED_RETURN_VALUE(notification_send(&hvx_params,
		                                                                  conn_handles.conn_handles[i]));
		                }
		       }
		 }
	    }   
    }
    
    
    return err_code;
    
}

ret_code_t ble_eco_pressure_update(ble_eco_t * p_eco,  uint16_t conn_handle)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;
    
    NRF_LOG_INFO("*p_eco->pressure_value:   %d deg C\n", *p_eco->pressure_value);
    
    if ( *p_eco->pressure_value!= p_eco->pressure_last)
    {
	    // Initialize value struct.
	    memset(&gatts_value, 0, sizeof(gatts_value));

	    gatts_value.len     = 4*sizeof(uint8_t);
	    gatts_value.offset  = 0;
	    gatts_value.p_value = p_eco->pressure_value;

	    // Update database.
	    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
		                                  p_eco->pressure_handles.value_handle,
		                                  &gatts_value);
	    
	    if (err_code == NRF_SUCCESS)
	    {
	    	p_eco->pressure_last = *p_eco->pressure_value;
	    }
	    else
	    {
	    	return err_code;
	    }
		            
	    ble_gatts_hvx_params_t hvx_params;
	    memset(&hvx_params, 0, sizeof(hvx_params));

	    hvx_params.handle = p_eco->pressure_handles.value_handle;
	    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	    hvx_params.offset = gatts_value.offset;
	    hvx_params.p_len  = &gatts_value.len;
	    hvx_params.p_data = gatts_value.p_value;
	    
	    NRF_LOG_INFO("hvx_params.p_data: %d", *hvx_params.p_data);
	    
	//    NRF_LOG_INFO("conn_handle: %d", (int)conn_handle);
	    if (conn_handle == BLE_CONN_HANDLE_ALL)
	    {
	    	ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();
	    	
	    	// Try sending notifications to all valid connection handles.
		for (uint32_t i = 0; i < conn_handles.len; i++)
		{
			if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
			{
				if (err_code == NRF_SUCCESS)
		                {
		                    err_code = notification_send(&hvx_params,
		                                                         conn_handles.conn_handles[i]);
		                }
		                else
		                {
		                    // Preserve the first non-zero error code
		                    UNUSED_RETURN_VALUE(notification_send(&hvx_params,
		                                                                  conn_handles.conn_handles[i]));
		                }
		       }
		 }
	    }   
    }
    
    
    return err_code;
    
}



/***********************************************************************************
*   Status register update
***********************************************************************************/

ret_code_t ble_eco_status_update(ble_eco_t * p_eco, uint16_t conn_handle)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;

    if (*p_eco->status_value != p_eco->status_last)
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = 2*sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = p_eco->status_value;

        // Update database.
        err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                          p_eco->status_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            p_eco->status_last = *p_eco->status_value;
        }
        else
        {
            return err_code;
        }

        // Send value if connected
        
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_eco->status_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

            if (conn_handle == BLE_CONN_HANDLE_ALL)
            {
                ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

                // Try sending notifications to all valid connection handles.
                for (uint32_t i = 0; i < conn_handles.len; i++)
                {
                    if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
                    {
                        if (err_code == NRF_SUCCESS)
                        {
                            err_code = notification_send(&hvx_params,
                                                                 conn_handles.conn_handles[i]);
                        }
                        else
                        {
                            // Preserve the first non-zero error code
                            UNUSED_RETURN_VALUE(notification_send(&hvx_params,
                                                                          conn_handles.conn_handles[i]));
                        }
                    }
                }
            }
            else
            {
                err_code = notification_send(&hvx_params, conn_handle);
            }
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }

    return err_code;
}

/*
*     Reconnection
*/

ret_code_t ble_eco_concentration_on_reconnection_update(ble_eco_t * p_eco,
                                                      uint16_t    conn_handle)
{
    if (p_eco == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t err_code;

    
        ble_gatts_hvx_params_t hvx_params;
        uint16_t               len = sizeof(uint8_t);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_eco->concentration_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = &p_eco->concentration_last;

        err_code = notification_send(&hvx_params, conn_handle);

        return err_code;

    return NRF_ERROR_INVALID_STATE;
}
#endif // NRF_MODULE_ENABLED(BLE_ECO)
