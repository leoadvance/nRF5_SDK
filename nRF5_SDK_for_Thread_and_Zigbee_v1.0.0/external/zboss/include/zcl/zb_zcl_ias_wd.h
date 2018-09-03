/**
 * ZBOSS Zigbee 3.0
 * 
 * Copyright (c) 2012-2018 DSR Corporation, Denver CO, USA.
 * http://www.dsr-zboss.com
 * http://www.dsr-corporation.com
 * 
 * All rights reserved.
 * 
 * 
 * Use in source and binary forms, redistribution in binary form only, with
 * or without modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 2. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 3. This software, with or without modification, must only be used with a Nordic
 *    Semiconductor ASA integrated circuit.
 * 
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * PURPOSE: IAS WD cluster defintions
*/

#if ! defined ZB_ZCL_IAS_WD_H
#define ZB_ZCL_IAS_WD_H

#include "zcl/zb_zcl_common.h"
#include "zcl/zb_zcl_commands.h"


/** @cond DOXYGEN_ZCL_SECTION */

/** @addtogroup ZB_ZCL_IAS_WD
 *  @{
 *    @details
 *    IAS WD cluster definitions
 */


/* Cluster ZB_ZCL_CLUSTER_ID_IAS_WD */

/*! @name IAS WD cluster attributes
    @{
*/

/*! @brief IAS WD cluster attribute identifiers
    @see ZCL spec, IAS WD Cluster 8.4.2.2
*/
enum zb_zcl_ias_wd_attr_e
{
  /*! @brief Max Duration attribute, ZCL spec 8.4.2.2.1 */
  ZB_ZCL_ATTR_IAS_WD_MAX_DURATION_ID = 0x0000,

};

/** @brief Max Duration attribute default value */
#define ZB_ZCL_ATTR_IAS_WD_MAX_DURATION_DEF_VALUE            240

/** @brief Max Duration attribute minimum value */
#define ZB_ZCL_ATTR_IAS_WD_MAX_DURATION_MIN_VALUE            0

/** @brief Max Duration attribute maximum value */
#define ZB_ZCL_ATTR_IAS_WD_MAX_DURATION_MAX_VALUE            0xfffe

/*! @internal @name IAS WD cluster internals
    Internal structures for attribute representation in cluster definitions.
    @{
*/

#define ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_IAS_WD_MAX_DURATION_ID(data_ptr) \
{                                                       \
  ZB_ZCL_ATTR_IAS_WD_MAX_DURATION_ID,                   \
  ZB_ZCL_ATTR_TYPE_U16,                                 \
  ZB_ZCL_ATTR_ACCESS_READ_WRITE,                        \
  (zb_voidp_t) data_ptr                                 \
}

/** @internal @brief Declare attribute list for IAS WD cluster - server side
    @param attr_list - attribure list name
    @param max_duration - pointer to variable to store Max Duration attribute
*/
#define ZB_ZCL_DECLARE_IAS_WD_ATTRIB_LIST(attr_list,                          \
    max_duration)                                                             \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST(attr_list)                                 \
  ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_IAS_WD_MAX_DURATION_ID, (max_duration))    \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST


/*! @internal Number of attributes mandatory for reporting in IAS WD cluster */
#define ZB_ZCL_IAS_WD_REPORT_ATTR_COUNT 0

/*! @} */ /* IAS WD cluster internals */

/*! @} */ /* IAS WD cluster attributes */

/*! @name IAS WD cluster commands
    @{
*/

/*! @brief IAS WD cluster command identifiers
    @see ZCL spec, IAS WD Cluster, 8.4.2.3
*/
enum zb_zcl_ias_wd_cmd_e
{
  ZB_ZCL_CMD_IAS_WD_START_WARNING_ID  = 0x00,  /**< Start warning command. ZCL spec 8.4.2.3.1 */
  ZB_ZCL_CMD_IAS_WD_SQUAWK_ID         = 0x01   /**< Squawk command. ZCL spec 8.4.2.3.2 */
};

/******************************* Start warning command ******************************/

/** @brief Warning Mode Field, see ZCL spec 8.4.2.3.1.2 */
enum zb_zcl_ias_wd_warning_mode_e
{
  /** Stop */
  ZB_ZCL_IAS_WD_WARNING_MODE_STOP      = 0x00,
  /** Burglar */
  ZB_ZCL_IAS_WD_WARNING_MODE_BURGLAR   = 0x01,
  /** Fire */
  ZB_ZCL_IAS_WD_WARNING_MODE_FIRE      = 0x02,
  /** Emergency */
  ZB_ZCL_IAS_WD_WARNING_MODE_EMERGENCY = 0x03,
};

/** @brief Strobe Field, see ZCL spec 8.4.2.3.1.3 */
enum zb_zcl_ias_wd_strobe_e
{
  /** No strobe */
  ZB_ZCL_IAS_WD_STROBE_NO_STROBE  = 0x00,
  /** Use strobe in parallel to warning */
  ZB_ZCL_IAS_WD_STROBE_USE_STROBE = 0x01,
};

/*! @brief Structure representsation Start warning command. ZCL spec 8.4.2.3.1 */
typedef ZB_PACKED_PRE struct zb_zcl_ias_wd_start_warning_s
{
  /** Status - enum flags Warning mode and Strobe, see ZCL spec 8.4.2.3.1 */
  zb_uint8_t status;
  /** Warning duration*/
  zb_uint16_t duration;
} ZB_PACKED_STRUCT zb_zcl_ias_wd_start_warning_t;

/*! @brief Get Warning Mode from payload Start warning command */
#define ZB_ZCL_IAS_WD_GET_WARNING_MODE(status)  ((status) & 0x0f)

/*! @brief Get Strobe from payload Start warning command */
#define ZB_ZCL_IAS_WD_GET_STROBE(status)        (((status) >>4) & 0x03)

/*! @brief Make Status field Start warning command from Warning Mode and Strobe */
#define ZB_ZCL_IAS_WD_MAKE_START_WARNING_STATUS(warning_mode, strobe)  \
  ((strobe) << 4 | (warning_mode))

/*! @brief Start warning command. ZCL spec 8.4.2.3.1
    @param buffer - to put packet to
    @param addr - address to send packet to
    @param dst_addr_mode - addressing mode
    @param dst_ep - destination endpoint
    @param ep - sending endpoint
    @param prfl_id - profile identifier
    @param def_resp - enable/disable default response
    @param cb - callback for getting command send status
    @param status - Status, see @ref zb_zcl_ias_wd_start_warning_s
    @param duration - Warning duration
*/
#define ZB_ZCL_IAS_WD_SEND_START_WARNING_REQ(                 \
    buffer, addr, dst_addr_mode, dst_ep, ep, prfl_id, def_resp, cb, \
    status, duration)                                         \
{                                                             \
  zb_uint8_t* ptr = ZB_ZCL_START_PACKET_REQ(buffer)              \
  ZB_ZCL_CONSTRUCT_SPECIFIC_COMMAND_REQ_FRAME_CONTROL(ptr, (def_resp))     \
  ZB_ZCL_CONSTRUCT_COMMAND_HEADER_REQ(ptr, ZB_ZCL_GET_SEQ_NUM(),  \
      ZB_ZCL_CMD_IAS_WD_START_WARNING_ID);                    \
  ZB_ZCL_PACKET_PUT_DATA8(ptr, (status));                     \
  ZB_ZCL_PACKET_PUT_DATA16_VAL(ptr, (duration));              \
  ZB_ZCL_FINISH_PACKET((buffer), ptr)                        \
  ZB_ZCL_SEND_COMMAND_SHORT(                                  \
      buffer, addr, dst_addr_mode, dst_ep, ep, prfl_id,       \
      ZB_ZCL_CLUSTER_ID_IAS_WD, cb);                          \
}

/** @brief Macro for getting Start warning command. ZCL spec 8.4.2.3.1
  * @attention Assumes that ZCL header already cut.
  * @param data_ptr - pointer to a variable of type @ref
  * zb_zcl_ias_wd_start_warning_s.
  * @param buffer containing the packet (by pointer).
  * @param status - variable to put parse status to (see @ref zb_zcl_parse_status_t).
  */
#define ZB_ZCL_IAS_WD_GET_START_WARNING_REQ(data_ptr, buffer, status)       \
{                                                                           \
  if (ZB_BUF_LEN((buffer)) != sizeof(zb_zcl_ias_wd_start_warning_t))        \
  {                                                                         \
   (status) = ZB_ZCL_PARSE_STATUS_FAILURE;                                  \
  }                                                                         \
  else                                                                      \
  {                                                                         \
    zb_zcl_ias_wd_start_warning_t *src_ptr =                                \
         (zb_zcl_ias_wd_start_warning_t*)ZB_BUF_BEGIN((buffer));            \
    (status) = ZB_ZCL_PARSE_STATUS_SUCCESS;                                 \
    (data_ptr)->status = src_ptr->status;                                   \
    ZB_HTOLE16(&((data_ptr)->duration), &(src_ptr->duration));              \
  }                                                                         \
}

/******************************* Squawk command ******************************/

/** @brief Squawk Mode Field, see ZCL spec 8.4.2.3.2.2 */
enum zb_zcl_ias_wd_squawk_mode_e
{
  /** Notification sound for "System is armed" */
  ZB_ZCL_IAS_WD_SQUAWK_MODE_ARMED      = 0x00,
  /** Notification sound for "System is disarmed"  */
  ZB_ZCL_IAS_WD_SQUAWK_MODE_DISARMED   = 0x01,
};

/** @brief Strobe Field, see ZCL spec 8.4.2.3.2.3 */
enum zb_zcl_ias_wd_squawk_strobe_e
{
  /** No strobe*/
  ZB_ZCL_IAS_WD_SQUAWK_STROBE_NO_STROBE  = 0x00,
  /** Use strobe blink in parallel to squawk */
  ZB_ZCL_IAS_WD_SQUAWK_STROBE_USE_STROBE = 0x01,
};

/** @brief Squawk level Field, see ZCL spec 8.4.2.3.2.4 */
enum zb_zcl_ias_wd_squawk_level_e
{
  /** Low level sound */
  ZB_ZCL_IAS_WD_SQUAWK_LEVEL_LOW        = 0x00,
  /** Medium level sound */
  ZB_ZCL_IAS_WD_SQUAWK_LEVEL_MEDIUM     = 0x01,
  /** High level sound */
  ZB_ZCL_IAS_WD_SQUAWK_LEVEL_HIGH       = 0x02,
  /** Very High level sound */
  ZB_ZCL_IAS_WD_SQUAWK_LEVEL_VERY_HIGH  = 0x03,
};

/*! @brief Structure representsation Squawk command. ZCL spec 8.4.2.3.2 */
typedef ZB_PACKED_PRE struct zb_zcl_ias_wd_squawk_s
{
  /** Status - enum flags Squawk mode, Strobe and Squawk level, see ZCL spec 8.4.2.3.2 */
  zb_uint8_t status;
} ZB_PACKED_STRUCT zb_zcl_ias_wd_squawk_t;

/*! @brief Get Squawk Mode from payload Squawk command */
#define ZB_ZCL_IAS_WD_GET_SQUAWK_MODE(status)  ((status) & 0x0f)

/*! @brief Get Squawk Strobe from payload Squawk command */
#define ZB_ZCL_IAS_WD_GET_SQUAWK_STROBE(status)  (((status) >>4) & 0x01)

/*! @brief Get Squawk Level from payload Squawk command */
#define ZB_ZCL_IAS_WD_GET_SQUAWK_LEVEL(status)  (((status) >>6) & 0x03)

/*! @brief Make Status field Squawk command from Squawk mode, Strobe and Squawk level */
#define ZB_ZCL_IAS_WD_MAKE_SQUAWK_STATUS(mode, strobe, level)  \
  ((level) << 4 | (strobe) << 1 | (mode))

/*! @brief Squawk command. ZCL spec 8.4.2.3.2
    @param buffer - to put packet to
    @param addr - address to send packet to
    @param dst_addr_mode - addressing mode
    @param dst_ep - destination endpoint
    @param ep - sending endpoint
    @param prfl_id - profile identifier
    @param def_resp - enable/disable default response
    @param cb - callback for getting command send status
    @param status - Status, see @ref zb_zcl_ias_wd_squawk_s
*/
#define ZB_ZCL_IAS_WD_SEND_SQUAWK_REQ(                        \
    buffer, addr, dst_addr_mode, dst_ep, ep, prfl_id, def_resp, cb, \
    status)                                                   \
{                                                             \
  zb_uint8_t* ptr = ZB_ZCL_START_PACKET_REQ(buffer)              \
  ZB_ZCL_CONSTRUCT_SPECIFIC_COMMAND_REQ_FRAME_CONTROL(ptr, (def_resp))     \
  ZB_ZCL_CONSTRUCT_COMMAND_HEADER_REQ(ptr, ZB_ZCL_GET_SEQ_NUM(),  \
      ZB_ZCL_CMD_IAS_WD_SQUAWK_ID);                           \
  ZB_ZCL_PACKET_PUT_DATA8(ptr, (status));                     \
  ZB_ZCL_FINISH_PACKET((buffer), ptr)                        \
  ZB_ZCL_SEND_COMMAND_SHORT(                                  \
      buffer, addr, dst_addr_mode, dst_ep, ep, prfl_id,       \
      ZB_ZCL_CLUSTER_ID_IAS_WD, cb);                          \
}

/** @brief Macro for getting Squawk command. ZCL spec 8.4.2.3.2
  * @attention Assumes that ZCL header already cut.
  * @param data_ptr - pointer to a variable of type @ref
  * zb_zcl_ias_wd_squawk_s.
  * @param buffer containing the packet (by pointer).
  * @param status - variable to put parse status to (see @ref zb_zcl_parse_status_t).
  */
#define ZB_ZCL_IAS_WD_GET_SQUAWK_REQ(data_ptr, buffer, status)      \
{                                                                   \
  if (ZB_BUF_LEN((buffer)) != sizeof(zb_zcl_ias_wd_squawk_t))       \
  {                                                                 \
   (status) = ZB_ZCL_PARSE_STATUS_FAILURE;                          \
  }                                                                 \
  else                                                              \
  {                                                                 \
    zb_zcl_ias_wd_squawk_t *src_ptr =                               \
         (zb_zcl_ias_wd_squawk_t*)ZB_BUF_BEGIN((buffer));           \
    (status) = ZB_ZCL_PARSE_STATUS_SUCCESS;                         \
    (data_ptr)->status = src_ptr->status;                           \
  }                                                                 \
}

/*! @} */ /* IAS WD cluster commands */

/*! @} */ /* ZCL IAS WD cluster definitions */


/** @brief Declare run Start Warning command for User Application
*/
typedef struct zb_zcl_ias_wd_start_warning_value_param_s
{
  zb_uint8_t warning_mode;        /*!< Warning Mode */
  zb_uint8_t strobe;              /*!< Strobe */
  zb_uint16_t duration;           /*!< Duration */
} zb_zcl_ias_wd_start_warning_value_param_t;

/*! Struct for invoke User App & continue after */
typedef struct zb_zcl_ias_wd_start_warning_user_app_schedule_e
{
  zb_zcl_parsed_hdr_t cmd_info;         /**< Parameters for continue command,
                                              see @ref zb_zcl_parsed_hdr_s */
  zb_zcl_ias_wd_start_warning_value_param_t param; /**< User App command parameters,
                                        see @ref zb_zcl_ias_wd_start_warning_value_param_s */
} zb_zcl_ias_wd_start_warning_user_app_schedule_t;

#define ZB_ZCL_IAS_WD_START_WARNING_SCHEDULE_USER_APP(buffer, pcmd_info,                        \
    warningMode, strobe_, duration_)                                                            \
{                                                                                               \
  zb_zcl_ias_wd_start_warning_user_app_schedule_t* user_data =                                  \
          ZB_GET_BUF_PARAM((buffer), zb_zcl_ias_wd_start_warning_user_app_schedule_t);          \
  ZB_MEMMOVE(&(user_data->cmd_info), (pcmd_info), sizeof(zb_zcl_parsed_hdr_t));                 \
  user_data->param.warning_mode = (warningMode);                                                \
  user_data->param.strobe = (strobe_);                                                          \
  user_data->param.duration = (duration_);                                                      \
  ZB_SCHEDULE_CALLBACK(zb_zcl_ias_wd_start_warning_invoke_user_app, ZB_REF_FROM_BUF((buffer))); \
}

/** @brief Declare run Squawk command for User Application
*/
typedef struct zb_zcl_ias_wd_squawk_value_param_s
{
  zb_uint8_t squawk_mode;         /*!< Squawk Mode */
  zb_uint8_t strobe;              /*!< Strobe */
  zb_uint8_t squawk_level;        /*!< Squawk level */
} zb_zcl_ias_wd_squawk_value_param_t;

/*! Struct for invoke User App & continue after */
typedef struct zb_zcl_ias_wd_squawk_user_app_schedule_e
{
  zb_zcl_parsed_hdr_t cmd_info;         /**< Parameters for continue command,
                                              see @ref zb_zcl_parsed_hdr_s */
  zb_zcl_ias_wd_squawk_value_param_t param; /**< User App command parameters,
                                        see @ref zb_zcl_ias_wd_squawk_value_param_s */
} zb_zcl_ias_wd_squawk_user_app_schedule_t;

#define ZB_ZCL_IAS_WD_SQUAWK_SCHEDULE_USER_APP(buffer, pcmd_info,                         \
    squawkMode, strobe_, level)                                                           \
{                                                                                         \
  zb_zcl_ias_wd_squawk_user_app_schedule_t* user_data =                                   \
          ZB_GET_BUF_PARAM((buffer), zb_zcl_ias_wd_squawk_user_app_schedule_t);           \
  ZB_MEMMOVE(&(user_data->cmd_info), (pcmd_info), sizeof(zb_zcl_parsed_hdr_t));           \
  user_data->param.squawk_mode = (squawkMode);                                            \
  user_data->param.strobe = (strobe_);                                                    \
  user_data->param.squawk_level = (level);                                                \
  ZB_SCHEDULE_CALLBACK(zb_zcl_ias_wd_squawk_invoke_user_app, ZB_REF_FROM_BUF((buffer)));  \
}

/** @endcond */ /* DOXYGEN_ZCL_SECTION */

zb_void_t zb_zcl_ias_wd_init_server();
zb_void_t zb_zcl_ias_wd_init_client();
#define ZB_ZCL_CLUSTER_ID_IAS_WD_SERVER_ROLE_INIT zb_zcl_ias_wd_init_server
#define ZB_ZCL_CLUSTER_ID_IAS_WD_CLIENT_ROLE_INIT zb_zcl_ias_wd_init_client

#endif /* ! defined ZB_ZCL_IAS_WD_H */
