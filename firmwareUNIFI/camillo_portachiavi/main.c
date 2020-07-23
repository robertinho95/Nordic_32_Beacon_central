#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_lls.h"


#define DEVICE_NAME                     "Camillo TAG"                           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                500                                     /**< The advertising duration in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */


#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//GPIO Definition
#define BUTTON_TAG                      NRF_GPIO_PIN_MAP(1,6)                   /**< Define Button GPIO_PIN */
#define CONNECTIVITY_LED                NRF_GPIO_PIN_MAP(0,12)                  /**< Define Connectivity_led (LED blue) GPIO_PIN */
#define ALERT_LED                       NRF_GPIO_PIN_MAP(0,8)                   /**< Define ALERT_LED (LED red) GPIO_PIN */
#define BUZZ                            NRF_GPIO_PIN_MAP(0,10)                  /**< Define Buzz GPIO_PIN */

// Button management
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
APP_TIMER_DEF(m_push_button_timer_id);                                          /**< Timer for long press button detection, for sleep device in every cases*/
static int COUNT_PUSH = 0;                                                      /**< Count for long press button detection*/
#define PUSH_BUTTON_INTERVAL            APP_TIMER_TICKS(500)                    /**< Delay to increment COUNT_PUSH variable*/
#define time_to_long_press              3000                                    /**< Time to long press detection*/
APP_TIMER_DEF(m_doublepush_button_timer_id);                                    /**< Timer for reset possibility to double click*/
static bool state_timer_button = false;                                         /**< Variable to define double click event*/ 
#define DOUBLEPUSH_BUTTON_INTERVAL      APP_TIMER_TICKS(2000)                   /**< Delay to reset double click event detection*/

// Led management
#define SLOW_BLINK_INTERVAL             APP_TIMER_TICKS(500)                    /**< Timer for Slow flashing LED (turn On/Off LED) */
int state_led = 1;                                                              /**< Variable to turn LED On/Off */
APP_TIMER_DEF(m_connectivity_led_flashing_timer_id);                            /**< Timer for flashing LED in Disconnected state */
#define GREEN_POWER_INTERVAL            APP_TIMER_TICKS(3000)                   /**< Timer for green power LED (turn On/Off LED) */
APP_TIMER_DEF(m_connectivity_led_green_power_id);                               /**< Timer for green power LED in Disconnected state */

// Module instance
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
BLE_LLS_DEF(m_lls);                                                             /**< Link Loss Service module instance*/
#define INITIAL_LLS_ALERT_LEVEL         BLE_CHAR_ALERT_LEVEL_MILD_ALERT         /**< Link Loss Service initial alert level definition*/

// Connection device management
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static bool device_connected = false;                                           /**< Variable to define device connected */

static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_LINK_LOSS_SERVICE, BLE_UUID_TYPE_BLE}
};

static bool emergency_mode = false;                                             /**< Variable to define emergency state*/

// Utils functions
static void sleep_mode_enter();
static void advertising_start();
static void buttons_init();

/**@brief Function for handling write events to the LED characteristic. 
 *
 * @param[in] p_camillo_service     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void client_write_handler(uint16_t conn_handle, ble_lls_t * p_lls, uint8_t alert_mod_state)
{

//********    TO BE IMPLEMENTED FOR PEER INFORMATION WITH CLIENT   ***********//

//    if (alert_mod_state)
//    {
//        NRF_LOG_INFO("Client writes something");
//    }
//    else
//    {
//        NRF_LOG_INFO("Client writes something");
//    }

}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

/**@brief Function for slow flashing led handler in advertising mode.
 *
 * @param[in] p_context  UNUSED.
 */
static void connectivity_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    if (emergency_mode == 1)
      {
          nrf_gpio_pin_write(ALERT_LED,state_led);
          nrf_gpio_pin_write(BUZZ,state_led);
      }
    if (state_led == 1)
    {
      state_led=0;
    }else{
      state_led=1;
    }
    nrf_gpio_pin_write(CONNECTIVITY_LED,state_led);

}

/**@brief Function for turn off led handler for increase battery life.
 *
 * @param[in] p_context  UNUSED.
 */
static void green_power_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    if (state_led == 1)
    {
      state_led=0;
      nrf_gpio_pin_write(CONNECTIVITY_LED,state_led);
    }else{
      state_led=1;
      nrf_gpio_pin_write(CONNECTIVITY_LED,state_led);
      app_timer_stop(m_connectivity_led_green_power_id);
    }  
    
}

/**@brief Function for timeout handler in long press function.
 *
 * @param[in] p_context  UNUSED.
 */
static void push_button_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    COUNT_PUSH += 1;
    if (COUNT_PUSH > time_to_long_press/500)
    {
        nrf_gpio_pin_write(ALERT_LED,1);
        nrf_gpio_pin_write(BUZZ,0);
        app_timer_stop(m_connectivity_led_flashing_timer_id);
        nrf_gpio_pin_write(CONNECTIVITY_LED,0);
    }
                    
}

/**@brief Function for timeout handler in double click function.
 *
 * @param[in] p_context  UNUSED.
 */
static void doublepush_button_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_gpio_pin_write(CONNECTIVITY_LED,1);       
    state_timer_button = false;
                       
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_connectivity_led_flashing_timer_id, APP_TIMER_MODE_REPEATED, connectivity_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_connectivity_led_green_power_id, APP_TIMER_MODE_REPEATED, green_power_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_push_button_timer_id, APP_TIMER_MODE_REPEATED, push_button_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_doublepush_button_timer_id, APP_TIMER_MODE_SINGLE_SHOT, doublepush_button_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
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

/**@brief Function for the Signals alert event from Immediate Alert or Link Loss services.
 *
 * @param[in] alert_level  Requested alert level.
 */
static void alert_signal(uint8_t alert_level)
{
    ret_code_t err_code;

    switch (alert_level)
    {
        case BLE_CHAR_ALERT_LEVEL_NO_ALERT:
            NRF_LOG_INFO("No Alert.");
            break; // BLE_CHAR_ALERT_LEVEL_NO_ALERT

        case BLE_CHAR_ALERT_LEVEL_MILD_ALERT:
            NRF_LOG_INFO("Mild Alert.");
            nrf_gpio_pin_write(ALERT_LED,0);
            nrf_gpio_pin_write(BUZZ,0);
            emergency_mode = true;
            break; // BLE_CHAR_ALERT_LEVEL_MILD_ALERT

        case BLE_CHAR_ALERT_LEVEL_HIGH_ALERT:
            NRF_LOG_INFO("HIGH Alert.");
            break; // BLE_CHAR_ALERT_LEVEL_HIGH_ALERT

        default:
            // No implementation needed.
            break;
    }
}


static void on_lls_evt(ble_lls_t * p_lls, ble_lls_evt_t * p_evt)
{
    switch (p_evt -> evt_type)
    {
        case BLE_LLS_EVT_LINK_LOSS_ALERT:
            alert_signal(p_evt -> params.alert_level);
            break;
        default:
            break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    ble_lls_init_t lls_init;

    lls_init.evt_handler = on_lls_evt;
    lls_init.initial_alert_level = INITIAL_LLS_ALERT_LEVEL;
    lls_init.client_write_handler = client_write_handler;

    err_code = ble_lls_init(&m_lls, &lls_init);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        device_connected = false;
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
{
    nrf_gpio_pin_write(CONNECTIVITY_LED,1);
    nrf_gpio_pin_write(ALERT_LED,1);
    nrf_gpio_pin_write(BUZZ,1);
    ret_code_t err_code;
    
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");

            // Set LED state
            err_code = app_timer_start(m_connectivity_led_flashing_timer_id, SLOW_BLINK_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            
            break;

        case BLE_ADV_EVT_IDLE:
            if (emergency_mode == false)
            {
                sleep_mode_enter();
            } else {
                advertising_start();
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected with reason %d.", p_ble_evt->evt.gap_evt.params.disconnected.reason);
            // LED indication will be changed when advertising starts.
            device_connected = false;
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            // Stop advertising LED timer, turn on "connected state" LED
            err_code = app_timer_stop(m_connectivity_led_flashing_timer_id);
            APP_ERROR_CHECK(err_code);
            nrf_gpio_pin_write(CONNECTIVITY_LED,0);
            nrf_gpio_pin_write(BUZZ,1);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            device_connected = true;
            APP_ERROR_CHECK(err_code);
            // Set LED state
            state_led = 1;
            err_code = app_timer_start(m_connectivity_led_green_power_id, GREEN_POWER_INTERVAL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
                p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
                p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
           // Accept parameters requested by the peer.
           ble_gap_conn_params_t params;
           params = p_gap_evt->params.conn_param_update_request.conn_params;
           err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
           APP_ERROR_CHECK(err_code);

           NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
               p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
               p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);
        } break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectively set members to @ref BLE_GAP_DATA_LENGTH_AUTO.
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        {
           NRF_LOG_DEBUG("BLE_GATTS_EVT_SYS_ATTR_MISSING");
           err_code = sd_ble_gatts_sys_attr_set(p_gap_evt->conn_handle, NULL, 0, 0);
           APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE:
        {
            ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

            if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
            {
                // Ignore LL collisions.
                NRF_LOG_DEBUG("LL transaction collision during PHY update.");
                break;
            }

            ble_gap_phys_t phys = { 0 };
            phys.tx_phys = p_phy_evt->tx_phy;
            phys.rx_phys = p_phy_evt->rx_phy;

        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buzz and LEDs.
 */
static void output_gpio_init(void)
{
    ret_code_t err_code;

    nrf_gpio_cfg_output(ALERT_LED);
    nrf_gpio_pin_write(ALERT_LED, 1);
    nrf_gpio_cfg_output(CONNECTIVITY_LED);
    nrf_gpio_pin_write(CONNECTIVITY_LED, 1);
    nrf_gpio_cfg_output(BUZZ);
    nrf_gpio_pin_write(BUZZ, 1);
    
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case BUTTON_TAG:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && button_action == APP_BUTTON_PUSH && device_connected)
            {
                
                app_timer_start(m_doublepush_button_timer_id, DOUBLEPUSH_BUTTON_INTERVAL, NULL);                    
            }
            else if (m_conn_handle != BLE_CONN_HANDLE_INVALID && button_action == APP_BUTTON_RELEASE && device_connected)
            {
                
                if (state_timer_button == false)
                {
                    state_timer_button = true;
                    nrf_gpio_pin_write(CONNECTIVITY_LED,0);
                } else {
                    state_timer_button = false;
                    nrf_gpio_pin_write(BUZZ,0);
                    emergency_mode = false;
                    sleep_mode_enter();
                }
                
            }
            
            if (button_action == APP_BUTTON_PUSH)
            {
                err_code = app_timer_start(m_push_button_timer_id, PUSH_BUTTON_INTERVAL, NULL);
                APP_ERROR_CHECK(err_code);
            
            } else if (button_action == APP_BUTTON_RELEASE)
            {
                if (COUNT_PUSH > time_to_long_press/500)
                {
                    COUNT_PUSH = 0;
                    emergency_mode = false;
                    sleep_mode_enter();
                } else {
                    COUNT_PUSH = 0;
                    err_code = app_timer_stop(m_push_button_timer_id);
                    APP_ERROR_CHECK(err_code);
                }
            }

            break;
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

/**@brief Function for initializing buttons.
 */
void buttons_init()
{
    ret_code_t err_code;
    
    static app_button_cfg_t buttons[] =
        {
            {BUTTON_TAG, false, BUTTON_PULL, button_event_handler}
        };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
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


/**@brief Function for starting advertising.
 */
static void advertising_start()
{
    device_connected = false;
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
    output_gpio_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("Camillo START");

    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

