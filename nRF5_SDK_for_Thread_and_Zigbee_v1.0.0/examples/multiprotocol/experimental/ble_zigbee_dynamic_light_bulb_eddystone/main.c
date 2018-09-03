/**
 * Copyright (c) 2018 - 2018, Nordic Semiconductor ASA
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
 * @defgroup zigbee_examples_multiprotocol_beacon main.c
 * @{
 * @ingroup zigbee_examples
 * @brief Eddystone Beacon GATT Configuration Service + EID/eTLM sample application main file with Zigbee HA light bulb profile.
 *
 * This file contains the source code for an Eddystone
 * Beacon GATT Configuration Service + EID/eTLM sample application
 * and Zigbee configuration and initialisation.
 */
#include "zboss_api.h"
#include "zb_ha_dimmable_light.h"
#include "zb_error_handler.h"

#include "nrf_ble_es.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "es_app_config.h"

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_pwm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define IEEE_CHANNEL_MASK                 (1l << ZIGBEE_CHANNEL)                /**< Scan only one, predefined channel to find the coordinator. */
#define HA_DIMMABLE_LIGHT_ENDPOINT        10                                    /**< Device endpoint, used to receive light controlling commands. */
#define ERASE_PERSISTENT_CONFIG           ZB_FALSE                              /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */
#define BULB_PWM_NAME                     PWM1                                  /**< PWM instance used to drive dimmable light bulb. */
#define BULB_PWM_TIMER                    2                                     /**< Timer number used by PWM. */

/* Basic cluster attributes initial values. */
#define BULB_INIT_BASIC_APP_VERSION       01                                    /**< Version of the application software (1 byte). */
#define BULB_INIT_BASIC_STACK_VERSION     10                                    /**< Version of the implementation of the ZigBee stack (1 byte). */
#define BULB_INIT_BASIC_HW_VERSION        11                                    /**< Version of the hardware of the device (1 byte). */
#define BULB_INIT_BASIC_MANUF_NAME        "Nordic Semiconductor"                /**< Manufacturer name (32 bytes). */
#define BULB_INIT_BASIC_MODEL_ID          "Dimable_Light_with_beacon_v0.1"      /**< Model number assigned by manufacturer (32-bytes long string). */
#define BULB_INIT_BASIC_DATE_CODE         "20180416"                            /**< First 8 bytes specify the date of manufacturer of the device in ISO 8601 format (YYYYMMDD). Th rest (8 bytes) are manufacturer specific. */
#define BULB_INIT_BASIC_POWER_SOURCE      ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE   /**< Type of power sources available for the device. For possible values see section 3.2.2.2.8 of ZCL specification. */
#define BULB_INIT_BASIC_LOCATION_DESC     "Office desk"                         /**< Describes the physical location of the device (16 bytes). May be modified during commisioning process. */
#define BULB_INIT_BASIC_PH_ENV            ZB_ZCL_BASIC_ENV_UNSPECIFIED          /**< Describes the type of physical environment. For possible values see section 3.2.2.2.10 of ZCL specification. */

#define DEAD_BEEF                         0xDEADBEEF                            /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define ADV_LED_PIN                       BSP_BOARD_LED_0                       /**< Toggles when non-connectable or connectable advertisement is sent. */
#define CONNECTED_LED_PIN                 BSP_BOARD_LED_1                       /**< Is on when device has connected. */

#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_2                       /**< LED indicating that light switch successfully joind ZigBee network. */
#define BULB_LED                          BSP_BOARD_LED_3                       /**< LED immitaing dimmable light bulb. */

/* Declare endpoint for Dimmable Light device with scenes. */
#define ZB_HA_DECLARE_LIGHT_EP(ep_name, ep_id, cluster_list)                         \
  ZB_ZCL_DECLARE_DEVICE_SCENE_TABLE_RECORD_TYPE(                                     \
    zb_ha_light_scene_table_ ## ep_name ## _t,                                       \
    ZB_ZCL_SCENES_FIELDSETS_LENGTH_2(ZB_ZCL_CLUSTER_ID_ON_OFF,                       \
                                     ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL));              \
  ZB_ZCL_DEFINE_DEVICE_SCENE_TABLE(                                                  \
    zb_ha_light_scene_table_ ## ep_name ## _t,                                       \
    g_zb_ha_light_scene_table_ ## ep_name);                                          \
  ZB_ZCL_DECLARE_HA_DIMMABLE_LIGHT_SIMPLE_DESC(ep_name, ep_id,                       \
    ZB_HA_DIMMABLE_LIGHT_IN_CLUSTER_NUM, ZB_HA_DIMMABLE_LIGHT_OUT_CLUSTER_NUM);      \
  ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info## device_ctx_name,               \
                                     ZB_HA_DIMMABLE_LIGHT_REPORT_ATTR_COUNT);        \
  ZBOSS_DEVICE_DECLARE_LEVEL_CONTROL_CTX(cvc_alarm_info## device_ctx_name,           \
                                         ZB_HA_DIMMABLE_LIGHT_CVC_ATTR_COUNT);       \
  ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID,                   \
                              sizeof(zb_ha_light_scene_table_ ## ep_name ## _t),     \
                              &g_zb_ha_light_scene_table_ ## ep_name,                \
                              ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t),\
                              cluster_list,                                          \
                              (zb_af_simple_desc_1_1_t*)&simple_desc_##ep_name,      \
                              ZB_HA_DIMMABLE_LIGHT_REPORT_ATTR_COUNT,                \
                              reporting_info## device_ctx_name,                      \
                              ZB_HA_DIMMABLE_LIGHT_CVC_ATTR_COUNT,                   \
                              cvc_alarm_info## device_ctx_name)

#if !defined ZB_ROUTER_ROLE
#error Define ZB_ROUTER_ROLE to compile light bulb (Router) source code.
#endif


/* Basic cluster attributes. */
typedef struct
{
    zb_uint8_t zcl_version;
    zb_uint8_t app_version;
    zb_uint8_t stack_version;
    zb_uint8_t hw_version;
    zb_char_t  mf_name[32];
    zb_char_t  model_id[32];
    zb_char_t  date_code[16];
    zb_uint8_t power_source;
    zb_char_t  location_id[5];
    zb_uint8_t ph_env;
} bulb_device_basic_attr_t;

/* Identify cluster attributes. */
typedef struct
{
    zb_uint16_t identify_time;
    zb_uint8_t  commission_state;
} bulb_device_identify_attr_t;

/* ON/Off cluster attributes. */
typedef struct
{
    zb_bool_t   on_off;
    zb_bool_t   global_scene_ctrl;
    zb_uint16_t on_time;
    zb_uint16_t off_wait_time;
} bulb_device_on_off_attr_t;

/* Level Control cluster attributes. */
typedef struct
{
    zb_uint8_t  current_level;
    zb_uint16_t remaining_time;
} bulb_device_level_control_attr_t;

/* Scenes cluster attributes. */
typedef struct
{
    zb_uint8_t  scene_count;
    zb_uint8_t  current_scene;
    zb_uint8_t  scene_valid;
    zb_uint8_t  name_support;
    zb_uint16_t current_group;
} bulb_device_scenes_attr_t;

/* Groups cluster attributes. */
typedef struct
{
    zb_uint8_t name_support;
} bulb_device_groups_attr_t;

/* Main application customizable context. Stores all settings and static values. */
typedef struct
{
    bulb_device_basic_attr_t         basic_attr;
    bulb_device_identify_attr_t      identify_attr;
    bulb_device_scenes_attr_t        scenes_attr;
    bulb_device_groups_attr_t        groups_attr;
    bulb_device_on_off_attr_t        on_off_attr;
    bulb_device_level_control_attr_t level_control_attr;
} bulb_device_ctx_t;


APP_PWM_INSTANCE(BULB_PWM_NAME, BULB_PWM_TIMER);
static zb_ieee_addr_t    m_ieee_addr;
static bulb_device_ctx_t m_dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST_HA(identify_attr_list,
                                       &m_dev_ctx.identify_attr.identify_time,
                                       &m_dev_ctx.identify_attr.commission_state);


ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(groups_attr_list, &m_dev_ctx.groups_attr.name_support);

ZB_ZCL_DECLARE_SCENES_ATTRIB_LIST(scenes_attr_list,
                                  &m_dev_ctx.scenes_attr.scene_count,
                                  &m_dev_ctx.scenes_attr.current_scene,
                                  &m_dev_ctx.scenes_attr.current_group,
                                  &m_dev_ctx.scenes_attr.scene_valid,
                                  &m_dev_ctx.scenes_attr.name_support);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_HA_ADDS_FULL(basic_attr_list,
                                              &m_dev_ctx.basic_attr.zcl_version,
                                              &m_dev_ctx.basic_attr.app_version,
                                              &m_dev_ctx.basic_attr.stack_version,
                                              &m_dev_ctx.basic_attr.hw_version,
                                              &m_dev_ctx.basic_attr.mf_name,
                                              &m_dev_ctx.basic_attr.model_id,
                                              &m_dev_ctx.basic_attr.date_code,
                                              &m_dev_ctx.basic_attr.power_source,
                                              &m_dev_ctx.basic_attr.location_id,
                                              &m_dev_ctx.basic_attr.ph_env);

ZB_ZCL_DECLARE_ON_OFF_ATTRIB_LIST_ZLL(on_off_attr_list,
                                      &m_dev_ctx.on_off_attr.on_off,
                                      &m_dev_ctx.on_off_attr.global_scene_ctrl,
                                      &m_dev_ctx.on_off_attr.on_time,
                                      &m_dev_ctx.on_off_attr.off_wait_time);

ZB_ZCL_DECLARE_LEVEL_CONTROL_ATTRIB_LIST(level_control_attr_list,
                                         &m_dev_ctx.level_control_attr.current_level,
                                         &m_dev_ctx.level_control_attr.remaining_time);

ZB_HA_DECLARE_DIMMABLE_LIGHT_CLUSTER_LIST(dimmable_light_clusters,
                                          basic_attr_list,
                                          identify_attr_list,
                                          groups_attr_list,
                                          scenes_attr_list,
                                          on_off_attr_list,
                                          level_control_attr_list);

ZB_HA_DECLARE_LIGHT_EP(dimmable_light_ep,
                       HA_DIMMABLE_LIGHT_ENDPOINT,
                       dimmable_light_clusters);

ZB_HA_DECLARE_DIMMABLE_LIGHT_CTX(dimmable_light_ctx,
                                 dimmable_light_ep);


/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           3

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

#define LED_INTERVAL                    100


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


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.common_evt.conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.common_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONNECTED:
            bsp_board_led_on(CONNECTED_LED_PIN);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            bsp_board_led_off(CONNECTED_LED_PIN);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling Eddystone events.
 *
 * @param[in] evt Eddystone event to handle.
 */
static void on_es_evt(nrf_ble_es_evt_t evt)
{
    switch (evt)
    {
        case NRF_BLE_ES_EVT_ADVERTISEMENT_SENT:
            bsp_board_led_invert(ADV_LED_PIN);
            break;

        case NRF_BLE_ES_EVT_CONNECTABLE_ADV_STARTED:
            bsp_board_led_on(ADV_LED_PIN);
            break;

        case NRF_BLE_ES_EVT_CONNECTABLE_ADV_STOPPED:
            bsp_board_led_off(ADV_LED_PIN);
            break;

        default:
            break;
    }
}

/**@brief Function for handling button events from app_button IRQ
 *
 * @param[in] pin_no        Pin of the button for which an event has occured
 * @param[in] button_action Press or Release
 */
static void button_evt_handler(uint8_t pin_no, uint8_t button_action)
{
    if (button_action == APP_BUTTON_PUSH && pin_no == BUTTON_REGISTRATION)
    {
        nrf_ble_es_on_start_connectable_advertising();
    }
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

/**@brief Function for the GAP initialization.
*
* @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
*          the device. It also sets the permissions and appearance.
*/
static void gap_params_init(void)
{
   ret_code_t              err_code;
   ble_gap_conn_params_t   gap_conn_params;
   ble_gap_conn_sec_mode_t sec_mode;
   uint8_t                 device_name[] = APP_DEVICE_NAME;

   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

   err_code = sd_ble_gap_device_name_set(&sec_mode,
                                         device_name,
                                         strlen((const char *)device_name));
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


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = 1;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

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

    err_code = ble_conn_params_init(&cp_init);
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

/**@brief Function for initializing LEDs and a single PWM channel.
 */
static void leds_init(void)
{
    ret_code_t       err_code;
    app_pwm_config_t pwm_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, bsp_board_led_idx_to_pin(BULB_LED));

    /* Initialize all LEDs. */
    bsp_board_init(BSP_INIT_LEDS);

    /* Initialize PWM running on timer 1 in order to control dimmable light bulb. */
    err_code = app_pwm_init(&BULB_PWM_NAME, &pwm_cfg, NULL);
    APP_ERROR_CHECK(err_code);

    app_pwm_enable(&BULB_PWM_NAME);

    while (app_pwm_channel_duty_set(&BULB_PWM_NAME, 0, 99) == NRF_ERROR_BUSY)
    {
    }
}

/**
 * @brief Function for initializing the registation button
 *
 * @retval Values returned by @ref app_button_init
 * @retval Values returned by @ref app_button_enable
 */
static void button_init(void)
{
    ret_code_t              err_code;
    const uint8_t           buttons_cnt  = 1;
    static app_button_cfg_t buttons_cfgs[1] =
    {
        {
            .pin_no         = BUTTON_REGISTRATION,
            .active_state   = APP_BUTTON_ACTIVE_LOW,
            .pull_cfg       = NRF_GPIO_PIN_PULLUP,
            .button_handler = button_evt_handler
        }
    };

    err_code = app_button_init(buttons_cfgs, buttons_cnt, APP_TIMER_TICKS(100));
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Application Timer Module
 */
static void timer_init(void)
{
    uint32_t error_code = NRF_SUCCESS;
    error_code          = app_timer_init();
    APP_ERROR_CHECK(error_code);
}


/**@brief Function for initializing the Application Scheduler Module
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for setting the light bulb brightness.
  *
  * @param[in]   new_level   Light bulb brightness value.
 */
static void level_control_set_value(zb_uint16_t new_level)
{
    m_dev_ctx.level_control_attr.current_level = new_level;

    NRF_LOG_INFO("Set level value: %i", new_level);

    /* Scale level value: APP_PWM uses 0-100 scale, but ZigBee level control cluster uses values from 0 up to 255. */
    new_level = new_level * 100 / 256;

    /* Set the duty cycle - keep trying until PWM is ready. */
    while (app_pwm_channel_duty_set(&BULB_PWM_NAME, 0, new_level) == NRF_ERROR_BUSY)
    {
    }

    /* According to the table 7.3 of Home Automation Profile Specification v 1.2 rev 29, chapter 7.1.3. */
    if (new_level == 0)
    {
        m_dev_ctx.on_off_attr.on_off = ZB_FALSE;
    }
    else
    {
        m_dev_ctx.on_off_attr.on_off = ZB_TRUE;
    }
}

/**@brief Function for turning ON/OFF the light bulb.
 *
 * @param[in]   on   Boolean light bulb state.
 */
static void on_off_set_value(zb_bool_t on)
{
    m_dev_ctx.on_off_attr.on_off = on;

    NRF_LOG_INFO("Set ON/OFF value: %i", on);

    if (on)
    {
        level_control_set_value(m_dev_ctx.level_control_attr.current_level);
    }
    else
    {
        /* Turn LED off but do not update m_dev_ctx.level_control_attr.current_level value. */
        while (app_pwm_channel_duty_set(&BULB_PWM_NAME, 0, 0) == NRF_ERROR_BUSY)
        {
        }
    }
}

/**@brief Function for initializing all clusters attributes.
 */
static void bulb_clusters_attr_init(void)
{
    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = BULB_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = BULB_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          BULB_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          BULB_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          BULB_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = BULB_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          BULB_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time    = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
    m_dev_ctx.identify_attr.commission_state = ZB_ZCL_ATTR_IDENTIFY_COMMISSION_STATE_HA_ID_DEF_VALUE;

    /* On/Off cluster attributes data */
    m_dev_ctx.on_off_attr.on_off            = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;
    m_dev_ctx.on_off_attr.global_scene_ctrl = ZB_TRUE;
    m_dev_ctx.on_off_attr.on_time           = 0;
    m_dev_ctx.on_off_attr.off_wait_time     = 0;

    m_dev_ctx.level_control_attr.current_level  = ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE;
    m_dev_ctx.level_control_attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFFAULT_VALUE_ZLL;
    ZB_ZCL_LEVEL_CONTROL_SET_ON_OFF_VALUE(HA_DIMMABLE_LIGHT_ENDPOINT, m_dev_ctx.on_off_attr.on_off);
    ZB_ZCL_LEVEL_CONTROL_SET_LEVEL_VALUE(HA_DIMMABLE_LIGHT_ENDPOINT, m_dev_ctx.level_control_attr.current_level);
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[in]   param   Reference to ZigBee stack buffer used to pass received data.
 */
static zb_void_t zcl_device_cb(zb_uint8_t param)
{
    zb_uint8_t                       cluster_id;
    zb_uint8_t                       attr_id;
    zb_buf_t                       * p_buffer = ZB_BUF_FROM_REF(param);
    zb_zcl_device_callback_param_t * p_device_cb_param =
                     ZB_GET_BUF_PARAM(p_buffer, zb_zcl_device_callback_param_t);

    NRF_LOG_INFO("zcl_device_cb id %hd", p_device_cb_param->device_cb_id);

    /* Set default response value. */
    p_device_cb_param->status = RET_OK;

    switch (p_device_cb_param->device_cb_id)
    {
        case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
            NRF_LOG_INFO("Level control setting to %d", p_device_cb_param->cb_param.level_control_set_value_param.new_value);
            level_control_set_value(p_device_cb_param->cb_param.level_control_set_value_param.new_value);
            break;

        case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
            cluster_id = p_device_cb_param->cb_param.set_attr_value_param.cluster_id;
            attr_id    = p_device_cb_param->cb_param.set_attr_value_param.attr_id;

            if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
            {
                uint8_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data8;

                NRF_LOG_INFO("on/off attribute setting to %hd", value);
                if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
                {
                    on_off_set_value((zb_bool_t) value);
                }
            }
            else if (cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
            {
                uint16_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data16;

                NRF_LOG_INFO("level control attribute setting to %hd", value);
                if (attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID)
                {
                    level_control_set_value(value);
                }
            }
            else
            {
                /* Other clusters can be processed here */
                NRF_LOG_INFO("Unhandled cluster attribute id: %d", cluster_id);
            }
            break;

        default:
            p_device_cb_param->status = RET_ERROR;
            break;
    }

    NRF_LOG_INFO("zcl_device_cb status: %hd", p_device_cb_param->status);
}

/**@brief Function for initializing the Zigbee Stack
 */
static void zigbee_init(void)
{
    uint64_t factoryAddress;

    /* Read long address from FICR. */
    factoryAddress = (uint64_t)NRF_FICR->DEVICEID[0] << 32;
    factoryAddress |= NRF_FICR->DEVICEID[1];
    memcpy(m_ieee_addr, &factoryAddress, sizeof(factoryAddress));

    /* Set ZigBee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize ZigBee stack. */
    ZB_INIT("led_bulb");

    /* Set static long IEEE address. */
    zb_set_long_address(&m_ieee_addr);
    zb_set_network_router_role(IEEE_CHANNEL_MASK);
    zb_set_max_children(0);
    zb_set_nvram_erase_at_start(ERASE_PERSISTENT_CONFIG);
    ZB_SET_KEEPALIVE_TIMEOUT(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register callback for handling ZCL commands. */
    zb_register_zboss_callback(ZB_ZCL_DEVICE_CB, SET_ZBOSS_CB(zcl_device_cb));

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&dimmable_light_ctx);

    bulb_clusters_attr_init();
    level_control_set_value(m_dev_ctx.level_control_attr.current_level);

    ZB_ZCL_SCENES_CLEAR_SCENE_TABLE(g_zb_ha_light_scene_table_dimmable_light_ep);
}


/**@brief ZigBee stack event handler.
 *
 * @param[in]   param   Reference to ZigBee stack buffer used to pass arguments (signal).
 */
void zboss_signal_handler(zb_uint8_t param)
{
    zb_zdo_app_signal_type_t sig    = zb_get_app_signal(param, NULL);
    zb_ret_t                 status = ZB_GET_APP_SIGNAL_STATUS(param);

    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == RET_OK)
            {
                NRF_LOG_INFO("Joined network successfully");
                bsp_board_led_on(ZIGBEE_NETWORK_STATE_LED);
            }
            else
            {
                NRF_LOG_ERROR("Failed to join network. Status: %d", status);
                bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
                bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
            }
            break;

        case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            if (status != RET_OK)
            {
                NRF_LOG_WARNING("Production config is not present or invalid");
            }
            break;

        default:
            /* Unhandled signal. For more information see: zb_zdo_app_signal_type_e and zb_ret_e */
            NRF_LOG_INFO("Unhandled signal %d. Status: %d", sig, status);
            break;
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}


 /***************************************************************************************************
 * @section Main
 **************************************************************************************************/

 /**
 * @brief Function for application main entry.
 */
int main(void)
{
    zb_ret_t zb_err_code;

    log_init();
    timer_init();
    leds_init();
    button_init();
    scheduler_init();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    conn_params_init();
    nrf_ble_es_init(on_es_evt);

    zigbee_init();

    // Start execution.
    NRF_LOG_INFO("BLE Zigbee dynamic light bulb with eddystone example started.");

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start();
    ZB_ERROR_CHECK(zb_err_code);

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        app_sched_execute();
    }

    TRACE_DEINIT();
}

/**
 * @}
 */
