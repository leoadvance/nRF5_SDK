/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
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
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "nrf.h"
#include "nrf_sdm.h"
#include "boards.h"
#include "nrf_mesh_sdk.h"
#include "nrf_delay.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_assert.h"
#include "log.h"
#include "timer.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"

#include "config_client.h"
#include "simple_on_off_client.h"

#include "simple_hal.h"
#include "provisioner.h"

#include "SEGGER_RTT.h"

#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_nus.h"



#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}








/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
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
    uint32_t               err_code;
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
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    //uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
//    bsp_btn_ble_on_ble_evt(p_ble_evt);

}

static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define SERVER_COUNT (3)
#define CLIENT_COUNT (SERVER_COUNT + 1)

typedef enum
{
    DEVICE_STATE_PROVISIONING,
    DEVICE_STATE_RUNNING
} device_state_t;

/*****************************************************************************
 * Static data
 *****************************************************************************/

static const uint8_t m_netkey[NRF_MESH_KEY_SIZE] = NETKEY;
static const uint8_t m_appkey[NRF_MESH_KEY_SIZE] = APPKEY;

static dsm_handle_t m_netkey_handle;
static dsm_handle_t m_appkey_handle;
static dsm_handle_t m_devkey_handles[SERVER_COUNT];
static dsm_handle_t m_server_handles[SERVER_COUNT];
static dsm_handle_t m_group_handle;

static simple_on_off_client_t m_clients[CLIENT_COUNT];

static device_state_t m_device_state;
static uint16_t m_unprov_index;

/* Forward declarations */
static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src);

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void mesh_core_setup(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing softdevice\n");
#if defined(S130) || defined(S132) || defined(S140)
    nrf_clock_lf_cfg_t lfc_cfg = {NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM};
#elif defined(S110)
    nrf_clock_lfclksrc_t lfc_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#endif
    //mesh_softdevice_setup(lfc_cfg);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing mesh stack\n");
    nrf_mesh_init_params_t mesh_init_params = {
        .lfclksrc = lfc_cfg,
        .assertion_handler = mesh_assert_handler,
    };
    ERROR_CHECK(nrf_mesh_init(&mesh_init_params));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling mesh stack\n");
    ERROR_CHECK(nrf_mesh_enable());
}

static void access_setup(void)
{
    dsm_init();
    access_init();

    /* Initialize and enable all the models before calling ***_flash_config_load. */
    ERROR_CHECK(config_client_init(config_client_event_cb));

     for (uint32_t i = 0; i < CLIENT_COUNT; ++i)
    {
        m_clients[i].status_cb = client_status_cb;
        ERROR_CHECK(simple_on_off_client_init(&m_clients[i], i));
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting up access layer and models\n");
    /* Set and add local addresses and keys, if recovery fails. */
    dsm_local_unicast_address_t local_address = {PROVISIONER_ADDRESS, ACCESS_ELEMENT_COUNT};
    if (!dsm_flash_config_load() || !access_flash_config_load())
    {
        dsm_local_unicast_addresses_set(&local_address);
        dsm_address_publish_add(GROUP_ADDRESS, &m_group_handle);
        dsm_subnet_add(0, m_netkey, &m_netkey_handle);
        dsm_appkey_add(0, m_netkey_handle, m_appkey, &m_appkey_handle);
    }
    /* Setup SERVER_COUNT clients. Publish address is set to the corresponding server on provisioning complete, only add handles if recovery failed (this is checked by looking at the publish address of the model) */
    for (uint32_t i = 0; i < SERVER_COUNT; ++i)
    {
        dsm_handle_t address_handle = DSM_HANDLE_INVALID;
        if (NRF_SUCCESS == access_model_publish_address_get(m_clients[i].model_handle, &address_handle) &&  DSM_HANDLE_INVALID != address_handle)
        {
            provisioner_config_successful_cb();
        }
        else
        {
            ERROR_CHECK(access_model_application_bind(m_clients[i].model_handle, m_appkey_handle));
            ERROR_CHECK(access_model_publish_application_set(m_clients[i].model_handle, m_appkey_handle));
        }
    }

    /* Last client is set to publish to the GROUP_ADDRESS, only add handles if recovery failed. */
    dsm_handle_t address_handle = DSM_HANDLE_INVALID;
    if (NRF_SUCCESS != access_model_publish_address_get(m_clients[SERVER_COUNT].model_handle, &address_handle) ||
        DSM_HANDLE_INVALID == address_handle)
    {
        ERROR_CHECK(access_model_application_bind(m_clients[SERVER_COUNT].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_application_set(m_clients[SERVER_COUNT].model_handle, m_appkey_handle));
        ERROR_CHECK(access_model_publish_address_set(m_clients[SERVER_COUNT].model_handle, m_group_handle));
    }
}

static uint32_t server_index_get(const simple_on_off_client_t * p_client)
{
    uint32_t index = (((uint32_t) p_client - ((uint32_t) &m_clients[0]))) / sizeof(m_clients[0]);
    NRF_MESH_ASSERT(index < SERVER_COUNT);
    return index;
}

static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src)
{
    uint32_t server_index = server_index_get(p_self);
    switch (status)
    {
        case SIMPLE_ON_OFF_STATUS_ON:
            hal_led_pin_set(BSP_LED_0 + server_index, true);
            break;

        case SIMPLE_ON_OFF_STATUS_OFF:
            hal_led_pin_set(BSP_LED_0 + server_index, false);
            break;

        case SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY:
            hal_led_blink_ms(LEDS_MASK, 100, 6);
            break;

        default:
            NRF_MESH_ASSERT(false);
            break;
    }

    /* Set 4th LED on when all servers are on. */
    if (hal_led_pin_get(BSP_LED_0) &&
        hal_led_pin_get(BSP_LED_1) &&
        hal_led_pin_get(BSP_LED_2))
    {
        hal_led_pin_set(BSP_LED_3, true);
    }
    else
    {
        hal_led_pin_set(BSP_LED_3, false);
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    if (m_device_state != DEVICE_STATE_RUNNING)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Not in RUNNING state\n");
        return;
    }
    uint32_t status = NRF_SUCCESS;
    switch (button_number)
    {
        case 0:
        case 1:
        case 2:
            /* Invert LED. */
            status = simple_on_off_client_set(&m_clients[button_number],
                                               !hal_led_pin_get(BSP_LED_0 + button_number));
            break;
        case 3:
            /* Group message: invert all LEDs. */
            status = simple_on_off_client_set_unreliable(&m_clients[button_number],
                                                          !hal_led_pin_get(BSP_LED_0 + button_number), 3);
            break;
        default:
            break;

    }

    if (status == NRF_ERROR_INVALID_STATE ||
        status == NRF_ERROR_BUSY)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot send. Device is busy.\n");
        hal_led_blink_ms(LEDS_MASK, 50, 4);
    }
    else
    {
        ERROR_CHECK(status);
    }
}

/*****************************************************************************
 * Event callbacks from the provisioner
 *****************************************************************************/

void provisioner_config_successful_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u successful\n", m_unprov_index);

    hal_led_pin_set(BSP_LED_0 + m_unprov_index, false);

    if (m_unprov_index < (SERVER_COUNT - 1))
    {
        m_unprov_index++;
        provisioner_wait_for_unprov();
        hal_led_pin_set(BSP_LED_0 + m_unprov_index, true);
    }
    else
    {
        hal_led_blink_ms(LEDS_MASK, 100, 4);
        m_device_state = DEVICE_STATE_RUNNING;
    }
}

void provisioner_config_failed_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u failed\n", m_unprov_index);

    /* Delete key and address. */
    ERROR_CHECK(dsm_address_publish_remove(m_server_handles[m_unprov_index]));
    ERROR_CHECK(dsm_devkey_delete(m_devkey_handles[m_unprov_index]));
    provisioner_wait_for_unprov();
}

void provisioner_prov_complete_cb(const nrf_mesh_evt_prov_complete_t * p_prov_data)
{
    /* We should not get here if all servers are provisioned. */
    NRF_MESH_ASSERT(m_unprov_index < SERVER_COUNT);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning complete. Adding address 0x%04x.\n", p_prov_data->address);

    /* Add to local storage. */
    ERROR_CHECK(dsm_address_publish_add(p_prov_data->address, &m_server_handles[m_unprov_index]));
    ERROR_CHECK(dsm_devkey_add(p_prov_data->address, m_netkey_handle, p_prov_data->p_devkey, &m_devkey_handles[m_unprov_index]));

    /* Set publish address for the client to the corresponding server. */
    ERROR_CHECK(access_model_publish_address_set(m_clients[m_unprov_index].model_handle, m_server_handles[m_unprov_index]));

    /* Bind the device key to the configuration server and set the new node as the active server. */
    ERROR_CHECK(config_client_server_bind(m_devkey_handles[m_unprov_index]));
    ERROR_CHECK(config_client_server_set(m_devkey_handles[m_unprov_index], m_server_handles[m_unprov_index]));


    /* Move on to the configuration step. */
    provisioner_configure();
}

static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint32_t button_number = p_data[0] - '0';
    button_event_handler(button_number);
}

static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Control Client Demo -----\n");
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uint32_t err_code;
    m_device_state = DEVICE_STATE_PROVISIONING;

    hal_leds_init();
    hal_buttons_init(button_event_handler);

    /* Set the first LED */
    hal_led_pin_set(BSP_LED_0, true);

    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    mesh_core_setup();
    access_setup();
    provisioner_init();
    provisioner_wait_for_unprov();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    while(true)
    {
        int key = SEGGER_RTT_GetKey(); /* Returns -1 if there is no data. */
        if (key >= '0' && key <= '3')
        {
            uint32_t button_number = key - '0';
            button_event_handler(button_number);
        }
        nrf_mesh_process();
    }
}
