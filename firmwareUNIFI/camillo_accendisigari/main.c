#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_sdm.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_util.h"
#include "app_error.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_util.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_ble_scan.h"

// GPIO Definition
#define CONNECTIVITY_LED                NRF_GPIO_PIN_MAP(0,12)
#define MAIN_LED                        NRF_GPIO_PIN_MAP(0,8)
#define DEBUG_LED                       NRF_GPIO_PIN_MAP(1,9)
#define BUZZ                            NRF_GPIO_PIN_MAP(0,10)
#define BUTTON_CLI                      NRF_GPIO_PIN_MAP(1,6) 

// Button managment
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
APP_TIMER_DEF(m_push_button_timer_id);                                          /**< Timer for long press button detection, for sleep device in every cases*/
static int COUNT_PUSH = 0;                                                      /**< Count for long press button detection*/
#define PUSH_BUTTON_INTERVAL            APP_TIMER_TICKS(500)                    /**< Delay to increment COUNT_PUSH variable*/
#define time_to_long_press              3000                                    /**< Time to long press detection*/


// Led Management
#define SLOW_BLINK_INTERVAL             APP_TIMER_TICKS(500) // Timer for Slow flashing LED (turn On/Off LED)
APP_TIMER_DEF(m_connectivity_led_flashing_timer_id); // Timer for flashing LED in Disconnected state
int state_led = 1;

// Connection device management
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static bool device_connected = false;
static bool button_actived = false;

static bool emergency_mode = false;

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

static char const m_target_periph_name[] = "Camillo TAG";

#define SCAN_DURATION_WITELIST          3000

/**< Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t const m_scan_param =
{
    .active        = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = SCAN_DURATION_WITELIST,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
};

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

/**@brief Connection parameters requested for connection.*/
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t) MIN_CONN_INTERVAL,
    (uint16_t) MAX_CONN_INTERVAL,
    (uint16_t) SLAVE_LATENCY,
    (uint16_t) CONN_SUP_TIMEOUT
};

//Module instance
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                           /**< Scanning module instance. */

static bool     m_memory_access_in_progress;                        /**< Flag to keep track of ongoing operations on persistent memory. */
static void scan_start(void);

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    nrf_gpio_pin_write(CONNECTIVITY_LED,1);
    nrf_gpio_pin_write(BUZZ,1);
    nrf_gpio_pin_write(MAIN_LED,1);
    ret_code_t err_code;
    
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
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

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for slow flashing led handler in advertising mode.
 *
 * @param[in] p_context  UNUSED.
 */
static void connectivity_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    if (state_led == 1)
    {
      state_led=0;
    }else{
      state_led=1;
    }
    nrf_gpio_pin_write(CONNECTIVITY_LED,state_led);
    nrf_gpio_pin_write(BUZZ,state_led);
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
        nrf_gpio_pin_write(MAIN_LED,0);
        nrf_gpio_pin_write(CONNECTIVITY_LED,1);
        //************* TO BE IMPLEMENT FOR SAVE TAG MAC *****************//
    
    }    
                    
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
    
    err_code = app_timer_create(&m_push_button_timer_id, APP_TIMER_MODE_REPEATED, push_button_timeout_handler);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and LEDs.
 */
static void leds_init(void)
{
    ret_code_t err_code;

    bsp_board_init(BSP_INIT_LEDS);
    nrf_gpio_cfg_output(MAIN_LED);
    nrf_gpio_pin_write(MAIN_LED, 1);
    nrf_gpio_cfg_output(CONNECTIVITY_LED);
    nrf_gpio_pin_write(CONNECTIVITY_LED, 1);
    nrf_gpio_cfg_output(DEBUG_LED);
    nrf_gpio_pin_write(DEBUG_LED, 1);
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
        case BUTTON_CLI:
            if (button_action == APP_BUTTON_PUSH && device_connected==false)// && m_button_notification_enabled)
            {
                button_actived = true;
            }
            else if (button_action == APP_BUTTON_RELEASE && device_connected==false && button_actived == true)
            {
                nrf_gpio_pin_write(MAIN_LED,1);
                
                sleep_mode_enter();
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

/**@brief Function for initializing buttons.
 */
static void buttons_init()
{
    ret_code_t err_code;
    
    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_CLI, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            
            m_conn_handle = p_ble_evt -> evt.gap_evt.conn_handle;
            device_connected = true;

            // Stop advertising LED timer, turn on "connected state" LED
            err_code = app_timer_stop(m_connectivity_led_flashing_timer_id);
            APP_ERROR_CHECK(err_code);
            nrf_gpio_pin_write(CONNECTIVITY_LED,0);
            nrf_gpio_pin_write(BUZZ,0);
//            // Discover peer's services.
//            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
//            APP_ERROR_CHECK(err_code);
//
            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
            break;
        
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected, reason 0x%x.",
                         p_gap_evt->params.disconnected.reason);
            device_connected = false;

            if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                scan_start();
            }
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

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
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

        default:
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

    // Register handlers for BLE and SoC events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
            NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                         p_evt->conn_handle,
                         p_evt->params.att_mtu_effective);
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
            NRF_LOG_INFO("Data length for connection 0x%x updated to %d.",
                         p_evt->conn_handle,
                         p_evt->params.data_length);
        } break;

        default:
            break;
    }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    //ble_lbs_on_db_disc_evt(&m_ble_lls_c, p_evt);
}

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        {
            //on_whitelist_req();
            //m_whitelist_disabled = false;
            
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            break;
        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
            break;

        default:
          break;
    }
}

/**@brief Function for initialization scanning and setting filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.p_scan_param     = &m_scan_param;
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan,
                                           SCAN_NAME_FILTER,
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);

//    if (is_connect_per_addr)
//    {
//       err_code = nrf_ble_scan_filter_set(&m_scan,
//                                          SCAN_ADDR_FILTER,
//                                          m_target_periph_addr.addr);
//       APP_ERROR_CHECK(err_code);
//    }

    err_code = nrf_ble_scan_filters_enable(&m_scan,
                                           NRF_BLE_SCAN_ALL_FILTER,
                                           false);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function to start scanning.
 */
void scan_start(void)
{
    ret_code_t err_code;

    if (nrf_fstorage_is_busy(NULL))
    {
        m_memory_access_in_progress = true;
        return;
    }

    NRF_LOG_INFO("Starting scan.");

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_connectivity_led_flashing_timer_id, SLOW_BLINK_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}

int main(void)
{
    log_init();
    timers_init();
    power_management_init();
    leds_init();
    buttons_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
//    lls_c_init();
    scan_init();

    // Start execution.
    NRF_LOG_INFO("Camillo Accendisigari");
    scan_start();

//    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
