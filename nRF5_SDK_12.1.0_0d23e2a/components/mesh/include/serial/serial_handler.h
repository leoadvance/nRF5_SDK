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

#ifndef SERIAL_HANDLER_H__
#define SERIAL_HANDLER_H__

#include <stdint.h>
#include "serial_packet.h"
#include "nrf_mesh.h"

/**
 * @defgroup MESH_SERIAL_HANDLER Serial Handler
 * @ingroup MESH_SERIAL
 * @{
 */

/**
 * Initializes the serial handler module.
 */
void serial_handler_init(void);


/**
 * Set the callback function for application serial commands.
 *
 * @param[in] rx_cb Function pointer to a callback function that should be
 * called on each incoming application command.
 */
void serial_handler_app_cb_set(nrf_mesh_serial_app_rx_cb_t rx_cb);

/**
 * Handles an incoming serial command in the CONFIG range.
 *
 * @param[in] p_cmd A pointer to a command to handle.
 *
 * @retval NRF_SUCCESS The packet was handled successfully.
 * @retval NRF_ERROR_NULL The p_cmd parameter was NULL.
 * @retval NRF_ERROR_INVALID_ADDR The command is outside the CONFIG range.
 * @retval NRF_ERROR_FORBIDDEN The command can't be executed in the current
 * hardware configuration.
 */
void serial_handler_config_rx(const serial_packet_t* p_cmd);

/**
 * Handles an incoming serial command in the DFU range.
 *
 * @param[in] p_cmd A pointer to a command to handle.
 *
 * @retval NRF_SUCCESS The packet was handled successfully.
 * @retval NRF_ERROR_NULL The p_cmd parameter was NULL.
 * @retval NRF_ERROR_INVALID_ADDR The command is outside the DFU range.
 * @retval NRF_ERROR_FORBIDDEN The command can't be executed in the current
 * hardware configuration.
 */
void serial_handler_dfu_rx(const serial_packet_t* p_cmd);

/**
 * Handles an incoming serial command in the OPEN MESH range.
 *
 * @param[in] p_cmd A pointer to a command to handle.
 *
 * @retval NRF_SUCCESS The packet was handled successfully.
 * @retval NRF_ERROR_NULL The p_cmd parameter was NULL.
 * @retval NRF_ERROR_FORBIDDEN The command can't be executed in the current
 * hardware configuration.
 */
void serial_handler_openmesh_rx(const serial_packet_t* p_cmd);

/**
 * Handles an incoming serial command in the APPLICATION range.
 *
 * @param[in] p_cmd A pointer to a command to handle.
 *
 * @retval NRF_SUCCESS The packet was handled successfully.
 * @retval NRF_ERROR_NULL The p_cmd parameter was NULL.
 * @retval NRF_ERROR_INVALID_ADDR The command is outside the APPLICATION range.
 * @retval NRF_ERROR_FORBIDDEN The command can't be executed in the current
 * hardware configuration.
 */
void serial_handler_app_rx(const serial_packet_t* p_cmd);

/** @} */

#endif /* SERIAL_HANDLER_H__ */

