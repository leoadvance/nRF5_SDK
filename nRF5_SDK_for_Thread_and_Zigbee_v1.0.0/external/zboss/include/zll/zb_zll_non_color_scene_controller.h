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
 * PURPOSE: ZLL Non-color Scene controller device definition
*/

#if ! defined ZB_ZLL_NON_COLOR_SCENE_CONTR_H
#define ZB_ZLL_NON_COLOR_SCENE_CONTR_H

/**
 *  @defgroup ZB_ZLL_NON_COLOR_SCENE_CONTR Non-color scene controller
 *  @ingroup ZB_ZLL_DEVICES
 *  @addtogroup ZB_ZLL_NON_COLOR_SCENE_CONTR
 *  ZLL Non-color scene controller
 *  @{
    @details
    Non-color scene controller has 6 clusters (see spec 5.3.4): \n
        - @ref ZB_ZCL_BASIC \n
        - @ref ZB_ZCL_IDENTIFY \n
        - @ref ZB_ZCL_GROUPS \n
        - @ref ZB_ZCL_SCENES \n
        - @ref ZB_ZCL_ON_OFF \n
        - @ref ZB_ZCL_LEVEL_CONTROL

    Sample use ZLL Non-color scene controller

    @par Example
    Delcaration clusters
    @snippet non_color_scene_controller.c COMMON_DECLARATION
    Register device list
    @snippet non_color_scene_controller.c REGISTER
    Handler sample
    @snippet non_color_scene_controller.c HANDLER
    @par

    See ll/all_devices test
*/

#if defined ZB_ZLL_DEFINE_DEVICE_NON_COLOR_SCENE_CONTROLLER || defined DOXYGEN

#define ZB_ZLL_DEVICE_VER_NON_COLOR_SCENE_CONTROLLER 0

/**< Non-color scene controller input clusters number. */
#define ZB_ZLL_NON_COLOR_SCENE_CONTROLLER_IN_CLUSTER_NUM  7

/**< Non-color scene controller output clusters number. */
#define ZB_ZLL_NON_COLOR_SCENE_CONTROLLER_OUT_CLUSTER_NUM 1

/**
 *  @brief Declare cluster list for Non-color Scene Controller device.
 *  @param cluster_list_name [IN] - cluster list variable name.
 *  @param commissioning_cluster_role [IN] - determines Commissioning @ref zb_zcl_cluster_role_e
 *  "cluster role."
 */
#define ZB_ZLL_DECLARE_NON_COLOR_SCENE_CONTROLLER_CLUSTER_LIST(      \
  cluster_list_name, commissioning_cluster_role)                                    \
      zb_zcl_cluster_desc_t cluster_list_name[] =                                   \
      {                                                                             \
        ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_BASIC,          0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID ), \
        ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_IDENTIFY,       0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID ), \
        ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_ON_OFF,         0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID ), \
        ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_GROUPS,         0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID ), \
        ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,  0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID ), \
        ZB_ZCL_CLUSTER_DESC(ZB_ZCL_CLUSTER_ID_SCENES,         0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID ), \
        ZB_ZCL_CLUSTER_DESC(ZB_ZLL_CLUSTER_ID_COMMISSIONING,  0, NULL, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_MANUF_CODE_INVALID ), \
        ZB_ZCL_CLUSTER_DESC(ZB_ZLL_CLUSTER_ID_COMMISSIONING,  0, NULL, ZB_ZCL_CLUSTER_CLIENT_ROLE, ZB_ZCL_MANUF_CODE_INVALID), \
      }

/**
 *  @brief Declare simple descriptor for Non-color Scene Controller device.
 *  @param ep_name - endpoint variable name.
 *  @param ep_id [IN] - endpoint ID.
 *  @param in_clust_num [IN] - number of supported input clusters.
 *  @param out_clust_num [IN] - number of supported output clusters.
 *  @note in_clust_num, out_clust_num should be defined by numeric constants, not variables or any
 *  definitions, because these values are used to form simple descriptor type name.
 */
#define ZB_ZCL_DECLARE_NON_COLOR_SCENE_CONTROLLER_SIMPLE_DESC(                     \
  ep_name, ep_id, in_clust_num, out_clust_num)                                     \
      ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);                         \
      ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num)  simple_desc_##ep_name = \
      {                                                                            \
        ep_id,                                                                     \
        ZB_AF_ZLL_PROFILE_ID,                                                      \
        ZB_ZLL_NON_COLOR_SCENE_CONTROLLER_DEVICE_ID,                               \
        ZB_ZLL_DEVICE_VER_NON_COLOR_SCENE_CONTROLLER,                              \
        0,                                                                         \
        in_clust_num,                                                              \
        out_clust_num,                                                             \
        {                                                                          \
          ZB_ZCL_CLUSTER_ID_BASIC,                                                 \
          ZB_ZCL_CLUSTER_ID_IDENTIFY,                                              \
          ZB_ZCL_CLUSTER_ID_ON_OFF,                                                \
          ZB_ZCL_CLUSTER_ID_GROUPS,                                                \
          ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,                                         \
          ZB_ZCL_CLUSTER_ID_SCENES,                                                \
          ZB_ZLL_CLUSTER_ID_COMMISSIONING,                                         \
          ZB_ZLL_CLUSTER_ID_COMMISSIONING                                          \
        }                                                                          \
      }

/**
 *  @brief Declare endpoint for Non-color Scene Controller device.
 *  @param ep_name [IN] - endpoint variable name.
 *  @param ep_id [IN] - endpoint ID.
 *  @param cluster_list [IN] - endpoint cluster list.
 */
#define ZB_ZLL_DECLARE_NON_COLOR_SCENE_CONTROLLER_EP(ep_name, ep_id, cluster_list) \
  ZB_ZCL_DECLARE_NON_COLOR_SCENE_CONTROLLER_SIMPLE_DESC(                \
    ep_name,                                                            \
    ep_id,                                                              \
    ZB_ZLL_NON_COLOR_SCENE_CONTROLLER_IN_CLUSTER_NUM,                   \
    ZB_ZLL_NON_COLOR_SCENE_CONTROLLER_OUT_CLUSTER_NUM);                 \
  ZB_AF_DECLARE_ENDPOINT_DESC(                                          \
    ep_name,                                                            \
    ep_id,                                                              \
    ZB_AF_ZLL_PROFILE_ID,                                               \
    0,                                                                  \
    NULL,                                                               \
    ZB_ZCL_ARRAY_SIZE(                                                  \
      cluster_list,                                                     \
      zb_zcl_cluster_desc_t),                                           \
    cluster_list,                                                       \
    (zb_af_simple_desc_1_1_t*)&simple_desc_##ep_name,                   \
    0, NULL, /* No reporting ctx */                                     \
    0, NULL) /* No CVC ctx */

#define ZB_ZLL_DECLARE_NON_COLOR_SCENE_CONTROLLER_CTX(device_ctx, ep_name) \
  ZBOSS_DECLARE_DEVICE_CTX_1_EP(device_ctx, ep_name)

#endif /* defined ZB_ZLL_DEFINE_DEVICE_NON_COLOR_SCENE_CONTROLLER || defined DOXYGEN */

/**
 * @}
 */

#endif /* ! defined ZB_ZLL_NON_COLOR_SCENE_CONTR_H */
