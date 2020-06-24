/**
 * Copyright (c) 2009 - 2019, Nordic Semiconductor ASA
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
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "nrf_power.h"
#include "nrf_drv_clock.h"
#include "nrf_sdh_freertos.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advdata.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh.h"
#include "nrf_ble_scan.h"
#include "nrf_drv_power.h"

QueueHandle_t advQueue;
typedef enum { ADV_FOUND } osEvent_t;
typedef struct
{
  osEvent_t event;
	int rssi;
	int txPower;
} advMessage_t;

TaskHandle_t main_task_handle;
TaskHandle_t timer_handle;
static void main_task_function (void * pvParameter);
static void timer_function (void * pvParameter);
static void ble_stack_init(void);
static void advertising_init(void);
static void advertising_start(void* _param);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void scan_init(void);
static void scan_start(void* _param);
static void scan_evt_handler(scan_evt_t const * p_scan_evt);

/**
 * @brief Function for application main entry.
 */
int main(void){
	/* Initialize clock driver for better time accuracy in FREERTOS */
  nrf_drv_clock_init();
	
	// If nRF52 USB Dongle is powered from USB (high voltage mode),
  // GPIO output voltage is set to 1.8 V by default, which is not
  // enough to turn on green and blue LEDs. Therefore, GPIO voltage
  // needs to be increased to 3.0 V by configuring the UICR register.
	if(nrf_power_mainregstatus_get() == NRF_POWER_MAINREGSTATUS_HIGH){    // Configure UICR_REGOUT0 register only if it is set to default value.
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
        (UICR_REGOUT0_VOUT_1V8 << UICR_REGOUT0_VOUT_Pos)){
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_1V8 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
    }
  } 
  ret_code_t ret;
	ret = nrf_drv_power_init(NULL);
	APP_ERROR_CHECK(ret);
	
	/* Activate deep sleep mode */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /* Configure LED-pins as outputs */
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,10));	// Bottom left
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,13));	// Top left
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,15));	// Bottom right
	nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,2));		// Top right
	
	/* Create task for LED0 blinking with priority set to 2 */
	xTaskCreate(main_task_function, "MAIN", configMINIMAL_STACK_SIZE + 200, NULL, 2, &main_task_handle);
	advQueue = xQueueCreate(10, sizeof(advMessage_t));

	
	ble_stack_init();
	advertising_init();
  scan_init();
	
	nrf_sdh_freertos_init(advertising_start, NULL);

	/* Start FreeRTOS scheduler. */
	vTaskStartScheduler();
	
	while (true){
		
	}
}
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(40, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define SCAN_INTERVAL										MSEC_TO_UNITS(1000, UNIT_0_625_MS)
#define SCAN_WINDOW											MSEC_TO_UNITS(1000,  UNIT_0_625_MS)

#define APP_BEACON_INFO_LENGTH          0x18                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_TX_POWER		 	              (-20)															 		//*< Supported tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */




#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
	.adv_data =
	{
		.p_data = m_enc_advdata,
		.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
	},
	.scan_rsp_data =
	{
		.p_data = NULL,
		.len    = 0
	}
};


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
	APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
											 // implementation.
	APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
											 // manufacturer specific data in this implementation.
	APP_BEACON_UUID,     // 128 bit UUID value.
	APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
	APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
	APP_MEASURED_RSSI,    // Manufacturer specific information. The Beacon's measured TX power in
											 // this implementation.
	(uint8_t)APP_TX_POWER
};

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param =
{
    .active        = 0x01,
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ALL_NOT_RESOLVED_DIRECTED,
    .timeout       = 0,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
};

/** @} */
/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void){
	uint32_t      err_code;
	ble_advdata_t advdata;
	uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

	ble_advdata_manuf_data_t manuf_specific_data;

	manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
	// If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
	// UICR instead of using the default values. The major and minor values obtained from the UICR
	// are encoded into advertising data in big endian order (MSB First).
	// To set the UICR used by this example to a desired value, write to the address 0x10001080
	// using the nrfjprog tool. The command to be used is as follows.
	// nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
	// For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
	// the following command should be used.
	// nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
	uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
	uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

	uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

	m_beacon_info[index++] = MSB_16(major_value);
	m_beacon_info[index++] = LSB_16(major_value);

	m_beacon_info[index++] = MSB_16(minor_value);
	m_beacon_info[index++] = LSB_16(minor_value);
#endif

	manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
	manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

	// Build and set advertising data.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type             = BLE_ADVDATA_NO_NAME;
	advdata.flags                 = flags;
	advdata.p_manuf_specific_data = &manuf_specific_data;

	// Initialize advertising parameters (used when starting advertising).
	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
	m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
	m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
	m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
	m_adv_params.duration        = 0;       // Never time out.

	err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
	APP_ERROR_CHECK(err_code);
	
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, APP_TX_POWER);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void* _param){
//	ret_code_t err_code;

//	err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
//	APP_ERROR_CHECK(err_code);
	
	scan_start(NULL);
}


/**@brief Function to start scanning. */
static void scan_start(void* _param)
{
    ret_code_t ret;

    ret = nrf_ble_scan_params_set(&m_scan, &m_scan_param);
    APP_ERROR_CHECK(ret);
	
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
typedef struct{
	int sw_1;
	int sw_2;
} Dev;

typedef struct{
	Dev dev_1;
	Dev dev_2;
} Devices;
Devices devices;
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context){
	ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
	switch (p_ble_evt->header.evt_id){
		case BLE_GAP_EVT_ADV_REPORT:{
			// Search for advertising names.
		  const ble_gap_evt_adv_report_t *p_adv_report = &p_gap_evt->params.adv_report;
			const ble_data_t *data = &p_adv_report->data;
			uint8_t *mfd_data = ble_advdata_parse(data->p_data, data->len, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA);
		
			if(mfd_data != 0){
				if(mfd_data[4+0] ==  0x01 && \
					 mfd_data[4+1] ==  0x12 && \
					 mfd_data[4+2] ==  0x23 && \
					 mfd_data[4+3] ==  0x34 && \
					 mfd_data[4+4] ==  0x45 && \
					 mfd_data[4+5] ==  0x56 && \
					 mfd_data[4+6] ==  0x67 && \
					 mfd_data[4+7] ==  0x78 && \
					 mfd_data[4+8] ==  0x89 && \
					 mfd_data[4+9] ==  0x9a && \
					 mfd_data[4+10] == 0xab && \
					 mfd_data[4+11] == 0xbc && \
					 mfd_data[4+12] == 0xcd && \
					 mfd_data[4+13] == 0xde && \
					 mfd_data[4+14] == 0xef && \
					 mfd_data[4+15] == 0xf0){
					devices.dev_1.sw_1 = (mfd_data[4+21] & 0x01) ? 1 : 0;
					devices.dev_1.sw_2 = (mfd_data[4+21] & 0x10) ? 1 : 0;
				}
				if(mfd_data[4+0] ==  0x01 && \
					 mfd_data[4+1] ==  0x12 && \
					 mfd_data[4+2] ==  0x23 && \
					 mfd_data[4+3] ==  0x34 && \
					 mfd_data[4+4] ==  0x45 && \
					 mfd_data[4+5] ==  0x56 && \
					 mfd_data[4+6] ==  0x67 && \
					 mfd_data[4+7] ==  0x78 && \
					 mfd_data[4+8] ==  0x89 && \
					 mfd_data[4+9] ==  0x9a && \
					 mfd_data[4+10] == 0xab && \
					 mfd_data[4+11] == 0xbc && \
					 mfd_data[4+12] == 0xcd && \
					 mfd_data[4+13] == 0xde && \
					 mfd_data[4+14] == 0xef && \
					 mfd_data[4+15] == 0x02){
					devices.dev_2.sw_1 = (mfd_data[4+21] & 0x01) ? 1 : 0;
					devices.dev_2.sw_2 = (mfd_data[4+21] & 0x10) ? 1 : 0;
				}
			}
		}
			break;
		case BLE_GAP_EVT_TIMEOUT:
			scan_start(NULL);
			break;

		default:
			break;
	}
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void){
	ret_code_t          err_code;
	nrf_ble_scan_init_t init_scan;

	memset(&init_scan, 0, sizeof(init_scan));

	init_scan.connect_if_match = false;
	init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
	init_scan.p_scan_param = &m_scan_param;

	err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt){
	switch(p_scan_evt->scan_evt_id){
		 default:
			break;
	}
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void){
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);
	
	err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
	APP_ERROR_CHECK(err_code);
	
	err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
	APP_ERROR_CHECK(err_code);
		
  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
static void main_task_function (void * pvParameter){
  UNUSED_PARAMETER(pvParameter);
	while (true){
		if(devices.dev_1.sw_1)
			nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1,10)); 	// Bottom left
		else
			nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1,10)); // Bottom left
		
		if(devices.dev_1.sw_2)
			nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1,13)); 	// Top left
		else
			nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1,13)); // Top left
		
		if(devices.dev_2.sw_1)
			nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1,15)); 	// Bottom right
		else
			nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1,15)); // Bottom right
		
		if(devices.dev_2.sw_2)
			nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0,2)); 	// Top right
		else
			nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0,2)); // Top right
		
		vTaskDelay(100);
  }
}

