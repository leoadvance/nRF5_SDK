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
 * @defgroup zigbee_examples_touchlink_switch main.c
 * @{
 * @ingroup  zigbee_examples
 * @brief    Zigbee touchlink initiator with light switch profile.
 */
#include "zboss_api.h"
#include "zll/zb_zll_common.h"
#include "zb_error_handler.h"
#include <stdarg.h>

#include "nrf_drv_gpiote.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define IEEE_CHANNEL_MASK     ((1l << 11) | (1l << 15) | (1l << 20) | (1l << 25) | (1l << ZIGBEE_CHANNEL)) /**< Scan all ZLL primary channels and the destination network channel to find touchlink targets or the coordinator. */
#define ERASE_PERSISTENT_CONFIG             ZB_FALSE                                /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */
#define LIGHT_SWITCH_ENDPOINT               1                                       /**< Source endpoint used to control light bulb. */
#define DEFAULT_GROUP_ID                    0xB331                                  /**< Group ID, which will be used to control all light sources with a single command. */

#define BULB_LED_TOUCHLINK_IN_PROGRESS      BSP_BOARD_LED_3
#define BULB_LED_NEW_DEV_JOINED             BSP_BOARD_LED_2
#define LIGHT_SWITCH_BUTTON_TOUCHLINK       BSP_BOARD_BUTTON_2                      /**< Button ID used to initiate touchlink commissioning. */

#define ZIGBEE_NETWORK_STATE_LED            BSP_BOARD_LED_2                         /**< LED indicating that light switch successfully joind ZigBee network. */
#define LIGHT_SWITCH_BUTTON_OFF             BSP_BOARD_BUTTON_1                      /**< Button ID used to switch off the light bulb. */
#define LIGHT_SWITCH_BUTTON_ON              BSP_BOARD_BUTTON_0                      /**< Button ID used to switch on the light bulb. */

#define LIGHT_SWITCH_BUTTON_THRESHOLD       ZB_TIME_ONE_SECOND                      /**< Number of beacon intervals (usually 15.36 usec) the button should be pressed to dimm the light bulb. */
#define LIGHT_SWITCH_BUTTON_SHORT_POLL_TMO  ZB_MILLISECONDS_TO_BEACON_INTERVAL(50)  /**< Delay between button state checks used in order to detect button long press. */
#define LIGHT_SWITCH_BUTTON_LONG_POLL_TMO   ZB_MILLISECONDS_TO_BEACON_INTERVAL(300) /**< Time after which the button state is checked again to detect button hold - the dimm command is sent again. */

#define LIGHT_SWITCH_DIMM_STEP              15                                      /**< DIm step size - increases/decreses current level (range 0x000 - 0xfe). */
#define LIGHT_SWITCH_DIMM_TRANSACTION_TIME  2                                       /**< Trasnsition time for a single step operation in 0.1 sec units. 0xFFFF - immediate change. */

#define MATCH_NWK_ADDR_REQ_TIMEOUT          (5 * ZB_TIME_ONE_SECOND)                /**< Timeout for network address request query. */
#define TOUCHLINK_RETRIES                   5                                       /**< Number of touchlink commissioning atempts. */

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile
#endif

typedef struct
{
    zb_ieee_addr_t ieee_addr;
    zb_uint16_t    short_addr;
    zb_uint8_t     endpoint;
} light_bulb_info_t;

/* Variables used to recognize the type of button press. */
typedef struct
{
    zb_bool_t in_progress;
    zb_time_t timestamp;
} light_switch_button_t;

typedef struct
{
    light_bulb_info_t     pending_dev;
    light_switch_button_t button;
    uint8_t               touchlink_in_progress;
    uint8_t               touchlink_cnt;
    uint8_t               nwk_join_in_progress;
} light_switch_ctx_t;


static void try_rejoin_network(void);
static zb_void_t bulb_nwk_addr_req_timeout(zb_uint8_t param);

static light_switch_ctx_t m_device_ctx;
static zb_ieee_addr_t     m_ieee_addr;
static zb_uint8_t         m_nwk_key[16] = { 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67, 0x89, 0, 0, 0, 0, 0, 0, 0, 0}; /**< Constant network key. */

/* Declare cluster list for Dimmer Switch device. */
ZB_ZLL_DECLARE_NON_COLOR_SCENE_CONTROLLER_CLUSTER_LIST(non_color_scene_controller_clusters,
                                                       ZB_ZCL_CLUSTER_SERVER_ROLE);

/* Declare endpoint for Dimmer Switch device. */
ZB_ZLL_DECLARE_NON_COLOR_SCENE_CONTROLLER_EP(non_color_scene_controller_ep,
                                             LIGHT_SWITCH_ENDPOINT,
                                             non_color_scene_controller_clusters);

/* Declare application's device context (list of registered endpoints) for Dimmer Switch device. */
ZB_ZLL_DECLARE_NON_COLOR_SCENE_CONTROLLER_CTX(touchlink_switch_ctx, non_color_scene_controller_ep);


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function used to verify if the provided cluster list contains all required clusters.
 *
 * @param[in] p_cluster_list       Cluster list to be checked.
 * @param[in] cluster_list_len     The length of the checked cluster list.
 * @param[in] match_cluster_count  The length of required cluster IDs list.
 * @param[in] ...                  List of required cluster IDs (variable length).
 *
 * @return Positive number if all required clusters were found.
 */
static int match_clusters(uint16_t *p_cluster_list, uint16_t cluster_list_len, int match_cluster_count, ...)
{
    uint16_t i      = 0;
    uint16_t result = 0;
    va_list ap;

    va_start(ap, match_cluster_count);

    for (;i < match_cluster_count; i++)
    {
        uint16_t cluster_id = (uint16_t)va_arg(ap, int);
        uint16_t j = 0;

        while (cluster_id != p_cluster_list[j])
        {
            if (j < cluster_list_len - 1)
            {
                j += 1;
            }
            else
            {
                goto fail;
            }
        }

        result +=1;
    }

fail:
    va_end(ap);
    return (result == match_cluster_count);
}


/***************************************************************************************************
 * @section Zigbee stack related functions.
 **************************************************************************************************/


/**@brief Function for sending ON/OFF requests to the light bulb.
 *
 * @param[in]   param    Non-zero reference to ZigBee stack buffer that will be used to construct on/off request.
 * @param[in]   on_off   Requested state of the light bulb.
 */
static zb_void_t light_switch_send_on_off(zb_uint8_t param, zb_uint16_t on_off)
{
    zb_uint16_t   group_id = DEFAULT_GROUP_ID;
    zb_buf_t    * p_buf    = ZB_BUF_FROM_REF(param);
    zb_uint8_t    cmd_id;

    if (!m_device_ctx.touchlink_in_progress)
    {
        NRF_LOG_INFO("Send ON/OFF command: %d", on_off);

        cmd_id = on_off ? ZB_ZCL_CMD_ON_OFF_ON_ID : ZB_ZCL_CMD_ON_OFF_OFF_ID;
        ZB_ZCL_ON_OFF_SEND_REQ(p_buf,
                               group_id,
                               ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                               0,
                               LIGHT_SWITCH_ENDPOINT,
                               ZB_AF_HA_PROFILE_ID,
                               ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
                               cmd_id,
                               NULL);
    }
    else
    {
        NRF_LOG_INFO("Send ON/OFF command blocked by touchlink commissioning");
        if (param)
        {
            ZB_FREE_BUF_BY_REF(param);
        }
    }
}


/**@brief Function for sending step requests to the light bulb.
  *
  * @param[in]   param        Non-zero reference to ZigBee stack buffer that will be used to construct step request.
  * @param[in]   is_step_up   Boolean parameter selecting direction of step change.
  */
static zb_void_t light_switch_send_step(zb_uint8_t param, zb_uint16_t is_step_up)
{
    zb_uint16_t   group_id = DEFAULT_GROUP_ID;
    zb_buf_t    * p_buf    = ZB_BUF_FROM_REF(param);
    zb_uint8_t    step_dir;

    if (!m_device_ctx.touchlink_in_progress)
    {
        NRF_LOG_INFO("Send step level command: %d", is_step_up);

        step_dir = is_step_up ? ZB_ZCL_LEVEL_CONTROL_STEP_MODE_UP : ZB_ZCL_LEVEL_CONTROL_STEP_MODE_DOWN;
        ZB_ZCL_LEVEL_CONTROL_SEND_STEP_REQ(p_buf,
                                           group_id,
                                           ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
                                           0,
                                           LIGHT_SWITCH_ENDPOINT,
                                           ZB_AF_HA_PROFILE_ID,
                                           ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
                                           NULL,
                                           step_dir,
                                           LIGHT_SWITCH_DIMM_STEP,
                                           LIGHT_SWITCH_DIMM_TRANSACTION_TIME);
    }
    else
    {
        NRF_LOG_INFO("Send step level command blocked by touchlink commissioning");
        if (param)
        {
            ZB_FREE_BUF_BY_REF(param);
        }
    }
}


/**@breif Reset pending device data, touchlink state and retry counter. */
static void new_device_failed(void)
{
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_device_ctx, 0, sizeof(light_switch_ctx_t)));
    bsp_board_led_off(BULB_LED_TOUCHLINK_IN_PROGRESS);
}


/**@breif Indicate that new device has joined the network, reset pending device data,
 *        touchlink state and retry counter.
 */
static void new_device_joined(void)
{
    new_device_failed();
    bsp_board_led_invert(BULB_LED_NEW_DEV_JOINED);
}


/**@brief Function for sending add group request. As a result all light bulb's
 *        light controlling endpoints will participate in the same group.
 *
 * @param[in]   param   Non-zero reference to ZigBee stack buffer that will be used to construct find request.
 */
static zb_void_t add_group(zb_uint8_t param)
{
    zb_buf_t * p_buf = ZB_BUF_FROM_REF(param);

    NRF_LOG_INFO("Include device %xd, ep %d to the group %d", m_device_ctx.pending_dev.short_addr, m_device_ctx.pending_dev.endpoint, DEFAULT_GROUP_ID);

    ZB_ZCL_GROUPS_SEND_ADD_GROUP_REQ(p_buf,
                                     m_device_ctx.pending_dev.short_addr,
                                     ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                     m_device_ctx.pending_dev.endpoint,
                                     LIGHT_SWITCH_ENDPOINT,
                                     ZB_AF_HA_PROFILE_ID,
                                     ZB_ZCL_DISABLE_DEFAULT_RESPONSE,
                                     NULL,
                                     DEFAULT_GROUP_ID);
    new_device_joined();
}


/**@brief Function for sending remove all groups request.
 *
 * @param[in]   param   Non-zero reference to ZigBee stack buffer that will be used to construct find request.
 */
static zb_void_t remove_all_groups(zb_uint8_t param)
{
    zb_buf_t * p_buf = ZB_BUF_FROM_REF(param);

    NRF_LOG_INFO("Remove all groups from device %d, ep %d", m_device_ctx.pending_dev.short_addr, m_device_ctx.pending_dev.endpoint);

    ZB_ZCL_GROUPS_SEND_REMOVE_ALL_GROUPS_REQ(p_buf,
                                             m_device_ctx.pending_dev.short_addr,
                                             ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
                                             m_device_ctx.pending_dev.endpoint,
                                             LIGHT_SWITCH_ENDPOINT,
                                             ZB_AF_HA_PROFILE_ID,
                                             ZB_ZCL_ENABLE_DEFAULT_RESPONSE,
                                             add_group);
}


/**@brief Callback function receiving simple descriptor responses.
 *
 * @param[in]   param   Reference to ZigBee stack buffer used to pass received data.
 */
static zb_void_t find_light_bulb_cb(zb_uint8_t param)
{
    zb_buf_t                   * p_buf  = ZB_BUF_FROM_REF(param);
    zb_zdo_simple_desc_resp_t  * p_resp = (zb_zdo_simple_desc_resp_t*)ZB_BUF_BEGIN(p_buf);

    if (p_resp->hdr.status == ZB_ZDP_STATUS_SUCCESS)
    {
        /* Look for ON/OFF and level control input clusters. */
        if (match_clusters((zb_uint16_t *)p_resp->simple_desc.app_cluster_list,
                           p_resp->simple_desc.app_input_cluster_count,
                           2,
                           ZB_ZCL_CLUSTER_ID_ON_OFF,
                           ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL))
        {
            NRF_LOG_INFO("All required clusters on the newly connected device found.");
            ZB_SCHEDULE_CALLBACK(remove_all_groups, param);
            param = 0;
        }
    }

    if (param)
    {
        new_device_failed();
        ZB_FREE_BUF_BY_REF(param);
    }
}


/**@brief Function for sending simple descriptor request.
 *
 * @param[in]   param   Non-zero reference to ZigBee stack buffer that will be used to construct find request.
 */
zb_void_t find_light_bulb(zb_uint8_t param)
{
    zb_buf_t                 * p_buf = ZB_BUF_FROM_REF(param);
    zb_zdo_simple_desc_req_t * p_req;

    NRF_LOG_INFO("Send simple descriptor request to the node %hd, endpoint %d",
                 m_device_ctx.pending_dev.short_addr,
                 m_device_ctx.pending_dev.endpoint);

    ZB_BUF_INITIAL_ALLOC(p_buf, sizeof(zb_zdo_simple_desc_req_t), p_req);
    ZB_BZERO(p_req, sizeof(zb_zdo_simple_desc_req_t));

    p_req->nwk_addr = m_device_ctx.pending_dev.short_addr;
    p_req->endpoint = m_device_ctx.pending_dev.endpoint;

    if (zb_zdo_simple_desc_req(param, find_light_bulb_cb) == ZB_ZDO_INVALID_TSN)
    {
        NRF_LOG_WARNING("Failed to send simple descriptor request");
        ZB_SCHEDULE_CALLBACK(find_light_bulb, param);
    }
}


/**@brief Callback function receiving network address responses.
 *
 * @param[in]   param   Reference to ZigBee stack buffer used to pass received data.
 */
static zb_void_t bulb_nwk_addr_req_cb(zb_uint8_t param)
{
    zb_address_ieee_ref_t         addr_ref;
    zb_ret_t                      zb_err_code;
    zb_buf_t                    * p_buf  = ZB_BUF_FROM_REF(param);
    zb_zdo_nwk_addr_resp_head_t * p_resp = (zb_zdo_nwk_addr_resp_head_t *)ZB_BUF_BEGIN(p_buf);

    NRF_LOG_INFO("Found network address: %d, status %hd", p_resp->nwk_addr, p_resp->status);

    if (p_resp->status == ZB_ZDP_STATUS_SUCCESS)
    {
        ZB_LETOH64(m_device_ctx.pending_dev.ieee_addr, p_resp->ieee_addr);
        ZB_LETOH16(&m_device_ctx.pending_dev.short_addr, &p_resp->nwk_addr);
        zb_address_update(m_device_ctx.pending_dev.ieee_addr, m_device_ctx.pending_dev.short_addr, ZB_TRUE, &addr_ref);

        /* Stop request timeout timer. */
        zb_err_code = ZB_SCHEDULE_ALARM_CANCEL(bulb_nwk_addr_req_timeout, ZB_ALARM_ANY_PARAM);
        ZB_ERROR_CHECK(zb_err_code);

        /* The next step is to bind the Light control to the bulb */
        ZB_SCHEDULE_CALLBACK(find_light_bulb, param);
        param = 0;
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}


/**@brief Function for sending network address request to the just commissioned device.
 *
 * @param[in]   param   Non-zero reference to ZigBee stack buffer that will be used to construct find request.
 */
zb_void_t bulb_nwk_addr_req(zb_uint8_t param)
{
    zb_buf_t                    * p_buf = ZB_BUF_FROM_REF(param);
    zb_zdo_nwk_addr_req_param_t * p_req = ZB_GET_BUF_TAIL(p_buf, sizeof(zb_zdo_nwk_addr_req_param_t));

    NRF_LOG_INFO("Find network address for light bulb:");
    NRF_LOG_HEXDUMP_INFO(m_device_ctx.pending_dev.ieee_addr, sizeof(zb_ieee_addr_t));

    ZB_IEEE_ADDR_COPY(p_req->ieee_addr, m_device_ctx.pending_dev.ieee_addr);
    p_req->dst_addr     = ZB_NWK_BROADCAST_ALL_DEVICES;
    p_req->start_index  = 0;
    p_req->request_type = 0;

    if (zb_zdo_nwk_addr_req(param, bulb_nwk_addr_req_cb) == ZB_ZDO_INVALID_TSN)
    {
        NRF_LOG_WARNING("Failed to send network address request");
        ZB_SCHEDULE_CALLBACK(bulb_nwk_addr_req, param);
    }
}


/**@brief NWK address finding procedure timeout handler.
 *
 * @param[in]   param   Reference to ZigBee stack buffer that will be used to construct find request.
 */
static zb_void_t bulb_nwk_addr_req_timeout(zb_uint8_t param)
{
    zb_ret_t zb_err_code;

    if (param)
    {
        NRF_LOG_INFO("NWK address not found, try again");
        zb_err_code = ZB_SCHEDULE_CALLBACK(bulb_nwk_addr_req, param);
        ZB_ERROR_CHECK(zb_err_code);
        zb_err_code = ZB_SCHEDULE_ALARM(bulb_nwk_addr_req_timeout, 0, MATCH_NWK_ADDR_REQ_TIMEOUT);
        ZB_ERROR_CHECK(zb_err_code);
    }
    else
    {
        zb_err_code = ZB_GET_OUT_BUF_DELAYED(bulb_nwk_addr_req_timeout);
        ZB_ERROR_CHECK(zb_err_code);
    }
}


/**@brief Starts touchlink initiator procedure. */
static void start_touchlink_commissioning(void)
{
    /* Check if previous touchlink procedure is still in progress. */
    if (m_device_ctx.touchlink_in_progress)
    {
        return;
    }

    /* Check if device is currently trying to join coordinated network. */
    if (m_device_ctx.nwk_join_in_progress)
    {
        return;
    }

    /* Reset touchlink retries counter. */
    if (m_device_ctx.touchlink_cnt == 0)
    {
        m_device_ctx.touchlink_cnt = TOUCHLINK_RETRIES;
    }

    NRF_LOG_INFO("Start Touchlink commissioning as initiator");
    m_device_ctx.touchlink_in_progress = 1;
    bsp_board_led_on(BULB_LED_TOUCHLINK_IN_PROGRESS);
    bdb_start_top_level_commissioning(ZB_BDB_TOUCHLINK_COMMISSIONING);
}


/**@brief Try to rejoin the Zigbee network that the device has already joined in the past. */
static void try_rejoin_network(void)
{
    /* Check if previous touchlink procedure is still in progress. */
    if (m_device_ctx.touchlink_in_progress)
    {
        return;
    }

    /* Check if device is currently trying to join coordinated network. */
    if (m_device_ctx.nwk_join_in_progress)
    {
        return;
    }

    /* Do not try to rejoin the network if the device was not commissioned in the past. */
    if (zb_bdb_is_factory_new())
    {
        return;
    }

    NRF_LOG_INFO("Try to join previous network.");
    m_device_ctx.nwk_join_in_progress = 1;
    bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
}


/**@brief Restarts touchlink initiator procedure if retry counter is greater than 0. */
static void retry_touchlink_commissioning(void)
{
    if (m_device_ctx.touchlink_in_progress)
    {
        m_device_ctx.touchlink_in_progress = 0;
        m_device_ctx.touchlink_cnt--;

        /* Retry touchlink initiator procedure touchlink_cnt times. */
        if (m_device_ctx.touchlink_cnt)
        {
            start_touchlink_commissioning();
        }
        else
        {
            /* Reset LEDs and unlock other commands execution. */
            new_device_failed();
        }
    }
}


/**@brief Callback for detecting button press duration.
 *
 * @param[in]   button   BSP Button number that was pressed.
 */
static zb_void_t light_switch_button_handler(zb_uint8_t button)
{
    zb_time_t current_time;
    zb_bool_t short_expired;
    zb_bool_t on_off;
    zb_ret_t  zb_err_code;

    current_time = ZB_TIMER_GET();

    if (!m_device_ctx.button.in_progress)
    {
        return;
    }

    if (button == LIGHT_SWITCH_BUTTON_ON)
    {
        on_off = ZB_TRUE;
    }
    else if (button == LIGHT_SWITCH_BUTTON_OFF)
    {
        on_off = ZB_FALSE;
    }
    else
    {
        return;
    }

    if (ZB_TIME_SUBTRACT(current_time, m_device_ctx.button.timestamp) > LIGHT_SWITCH_BUTTON_THRESHOLD)
    {
        short_expired = ZB_TRUE;
    }
    else
    {
        short_expired = ZB_FALSE;
    }

    /* Check if button was released during LIGHT_SWITCH_BUTTON_SHORT_POLL_TMO. */
    if (nrf_drv_gpiote_in_is_set(bsp_board_button_idx_to_pin(button)))
    {
        if (!short_expired)
        {
            /* Allocate output buffer and send on/off command. */
            zb_err_code = ZB_GET_OUT_BUF_DELAYED2(light_switch_send_on_off, on_off);
            ZB_ERROR_CHECK(zb_err_code);
        }

        /* Button released - wait for next event. */
        m_device_ctx.button.in_progress = ZB_FALSE;
    }
    else
    {
        if (short_expired)
        {
            /* The button is still pressed - allocate output buffer and send step command. */
            zb_err_code = ZB_GET_OUT_BUF_DELAYED2(light_switch_send_step, on_off);
            ZB_ERROR_CHECK(zb_err_code);

            /* Check if button will be pressed after next LIGHT_SWITCH_BUTTON_LONG_POLL_TMO to resend step command. */
            zb_err_code = ZB_SCHEDULE_ALARM(light_switch_button_handler, button, LIGHT_SWITCH_BUTTON_LONG_POLL_TMO);
            ZB_ERROR_CHECK(zb_err_code);
        }
        else
        {
            /* Wait another LIGHT_SWITCH_BUTTON_SHORT_POLL_TMO, until LIGHT_SWITCH_BUTTON_THRESHOLD will be reached. */
            zb_err_code = ZB_SCHEDULE_ALARM(light_switch_button_handler, button, LIGHT_SWITCH_BUTTON_SHORT_POLL_TMO);
            ZB_ERROR_CHECK(zb_err_code);
        }
    }
}


/**@brief Callback for button events.
 *
 * @param[in]   pin      Pin that triggered this event.
 * @param[in]   action   Action that lead to triggering this event.
 */
static void buttons_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t button = bsp_board_pin_to_button_idx(pin);
    zb_ret_t zb_err_code;

    if (action != GPIOTE_CONFIG_POLARITY_HiToLo)
    {
        return;
    }

    if (button == LIGHT_SWITCH_BUTTON_TOUCHLINK)
    {
        start_touchlink_commissioning();
        return;
    }

    /* If device has not joined Zigbee network - initiate rejoin procedure instead of trying to send switching commands. */
    if (!ZB_JOINED())
    {
        try_rejoin_network();
        return;
    }

    if (!m_device_ctx.button.in_progress)
    {
        m_device_ctx.button.in_progress = ZB_TRUE;
        m_device_ctx.button.timestamp   = ZB_TIMER_GET();

        zb_err_code = ZB_SCHEDULE_ALARM(light_switch_button_handler, button, LIGHT_SWITCH_BUTTON_SHORT_POLL_TMO);
        ZB_ERROR_CHECK(zb_err_code);
    }
}


/**@brief Function for initializing LEDs and buttons. */
static void leds_buttons_init(void)
{
    nrf_drv_gpiote_in_config_t buttons_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    uint32_t                   error_code;

    /* Initialize LEDs - use bsp_board to control them. */
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_leds_off();

    /* Initialize buttons - use nrf_drv_gpioto directly to enable interrupts. */
    error_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(error_code);

    buttons_config.pull = NRF_GPIO_PIN_PULLUP;
    error_code = nrf_drv_gpiote_in_init(bsp_board_button_idx_to_pin(LIGHT_SWITCH_BUTTON_ON), &buttons_config, buttons_handler);
    APP_ERROR_CHECK(error_code);
    nrf_drv_gpiote_in_event_enable(bsp_board_button_idx_to_pin(LIGHT_SWITCH_BUTTON_ON), true);

    error_code = nrf_drv_gpiote_in_init(bsp_board_button_idx_to_pin(LIGHT_SWITCH_BUTTON_OFF), &buttons_config, buttons_handler);
    APP_ERROR_CHECK(error_code);
    nrf_drv_gpiote_in_event_enable(bsp_board_button_idx_to_pin(LIGHT_SWITCH_BUTTON_OFF), true);

    error_code = nrf_drv_gpiote_in_init(bsp_board_button_idx_to_pin(LIGHT_SWITCH_BUTTON_TOUCHLINK), &buttons_config, buttons_handler);
    APP_ERROR_CHECK(error_code);
    nrf_drv_gpiote_in_event_enable(bsp_board_button_idx_to_pin(LIGHT_SWITCH_BUTTON_TOUCHLINK), true);
}


/**@brief ZigBee stack event handler.
 *
 * @param[in]   param   Reference to ZigBee stack buffer used to pass arguments (signal).
 */
zb_void_t zboss_signal_handler(zb_uint8_t param)
{
    zb_zdo_app_signal_hdr_t *sg_p   = NULL;
    zb_zdo_app_signal_type_t sig    = zb_get_app_signal(param, &sg_p);
    zb_ret_t                 status = ZB_GET_APP_SIGNAL_STATUS(param);

    switch(sig)
    {
        case ZB_ZDO_SIGNAL_SKIP_STARTUP:
            NRF_LOG_INFO("Device started without commissioning procedure. Status: %d", status);

            /* Reset pending device context. */
            UNUSED_RETURN_VALUE(ZB_MEMSET(&m_device_ctx, 0, sizeof(light_switch_ctx_t)));

            /* Try to rejoin previous network using network steering. If device is factory new - wait for user to press touchlink button. */
            try_rejoin_network();
            break;

        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            NRF_LOG_INFO("Device RESTARTED. Joined to the previous network: %d", ZB_JOINED());
            m_device_ctx.nwk_join_in_progress = 0;

            if (ZB_JOINED())
            {
                new_device_joined();
            }
            else
            {
                new_device_failed();
            }
            break;

        case ZB_BDB_SIGNAL_TOUCHLINK_NWK_STARTED:
        {
            zb_bdb_signal_touchlink_nwk_started_params_t *sig_params = ZB_ZDO_SIGNAL_GET_PARAMS(sg_p, zb_bdb_signal_touchlink_nwk_started_params_t);
            ZB_IEEE_ADDR_COPY(m_device_ctx.pending_dev.ieee_addr, sig_params->device_ieee_addr);
            m_device_ctx.pending_dev.endpoint = sig_params->endpoint;

            NRF_LOG_INFO("New network started. First router: ");
            NRF_LOG_HEXDUMP_INFO(m_device_ctx.pending_dev.ieee_addr, sizeof(zb_ieee_addr_t));
            NRF_LOG_INFO("profile 0x%x ep %hd", sig_params->profile_id, sig_params->endpoint);
        }
        break;

        case ZB_BDB_SIGNAL_TOUCHLINK_NWK_JOINED_ROUTER:
        {
            zb_bdb_signal_touchlink_nwk_joined_router_t *sig_params = ZB_ZDO_SIGNAL_GET_PARAMS(sg_p, zb_bdb_signal_touchlink_nwk_joined_router_t);
            ZB_IEEE_ADDR_COPY(m_device_ctx.pending_dev.ieee_addr, sig_params->device_ieee_addr);
            m_device_ctx.pending_dev.endpoint = sig_params->endpoint;

            NRF_LOG_INFO("New router joined the network:");
            NRF_LOG_HEXDUMP_INFO(m_device_ctx.pending_dev.ieee_addr, sizeof(zb_ieee_addr_t));
            NRF_LOG_INFO("profile 0x%x ep %hd", sig_params->profile_id, sig_params->endpoint);
        }
        break;

        case ZB_BDB_SIGNAL_TOUCHLINK:
            NRF_LOG_INFO("Touchlink commissioning as initiator finished. Status: %d", status);

            if ((status == ZB_BDB_STATUS_SUCCESS) &&
                (!ZB_IEEE_ADDR_IS_ZERO(m_device_ctx.pending_dev.ieee_addr)))
            {
                zb_uint16_t short_addr = zb_address_short_by_ieee(m_device_ctx.pending_dev.ieee_addr);
                zb_ret_t    zb_err_code;

                NRF_LOG_INFO("Touchlink commissioning as initiator succeed. Query device network address.");

                if (short_addr == ZB_UNKNOWN_SHORT_ADDR)
                {
                    zb_err_code = ZB_SCHEDULE_CALLBACK(bulb_nwk_addr_req, param);
                    ZB_ERROR_CHECK(zb_err_code);
                    zb_err_code = ZB_SCHEDULE_ALARM(bulb_nwk_addr_req_timeout, 0, MATCH_NWK_ADDR_REQ_TIMEOUT);
                    ZB_ERROR_CHECK(zb_err_code);
                }
                else
                {
                    m_device_ctx.pending_dev.short_addr = short_addr;
                    zb_err_code = ZB_SCHEDULE_CALLBACK(find_light_bulb, param);
                    ZB_ERROR_CHECK(zb_err_code);
                }

                // Do not free buffer as it was passed throught Zigbee scheduler.
                param = 0;
            }
            else if (status == RET_BUSY)
            {
                NRF_LOG_ERROR("Unable to rejoin touchlink network.");
                new_device_failed();
            }
            else
            {
                retry_touchlink_commissioning();
            }
            break;

        default:
            NRF_LOG_INFO("Unknown signal %hd. Status %d", sig, status);
            break;
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}


/**@brief Function for application main entry. */
int main(void)
{
    zb_ret_t zb_err_code;
    uint64_t factoryAddress;

    /* Read long address from FICR. */
    factoryAddress = (uint64_t)NRF_FICR->DEVICEID[0] << 32;
    factoryAddress |= NRF_FICR->DEVICEID[1];
    memcpy(m_ieee_addr, &factoryAddress, sizeof(factoryAddress));

    /* Initialize loging system and GPIOs. */
    log_init();
    leds_buttons_init();

    /* Set ZigBee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize ZigBee stack. */
    ZB_INIT("touchlink_light_switch");

    /* Set up Zigbee protocol main parameters. */
    zb_set_long_address(&m_ieee_addr);
    zb_set_nvram_erase_at_start(ERASE_PERSISTENT_CONFIG);

    /* set up 802.15.4 channels used by this device. */
    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    zb_zdo_touchlink_set_nwk_channel(ZIGBEE_CHANNEL);

    ZB_SET_ED_TIMEOUT(ED_AGING_TIMEOUT_64MIN);
    ZB_SET_KEEPALIVE_TIMEOUT(ZB_MILLISECONDS_TO_BEACON_INTERVAL(1000));
    zb_set_rx_on_when_idle(ZB_TRUE);

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_device_ctx, 0, sizeof(light_switch_ctx_t)));

    /* Set up security keys. */
    zb_secur_setup_nwk_key(m_nwk_key, 0);

    /* Register touchlink switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&touchlink_switch_ctx);

    /* Start execution. */
    NRF_LOG_INFO("Zigbee light switch with touchlink initiator example started.");

    /* Start ZigBee stack, do not perform commissioning automatically, wait for button press. */
    zb_err_code = zboss_start_no_autostart();
    ZB_ERROR_CHECK(zb_err_code);

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }

    TRACE_DEINIT();
}


/**
 * @}
 */
