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
#include <nrf_error.h>
#include "log.h"
#if HOST
#include <stdio.h>
#endif

#if NRF_MESH_LOG_ENABLE

const char * g_log_hex_digits = "0123456789ABCDEF";

uint32_t       g_log_debug_msk = LOG_MSK_DEFAULT;
uint32_t       g_log_debug_level = LOG_LEVEL_DEFAULT;
log_callback_t m_log_callback = LOG_CALLBACK_DEFAULT;

#if (LOG_ENABLE_RTT && !HOST)
#include <SEGGER_RTT.h>

/* For some reason, this function, while not static, is not included in the RTT header files. */
int SEGGER_RTT_vprintf(unsigned, const char *, va_list *);

uint32_t log_callback_rtt(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments)
{
    SEGGER_RTT_printf(0, "<t: %10u>, %s, %4d, ",timestamp, p_filename, line);
    SEGGER_RTT_vprintf(0, format, &arguments);

    return NRF_SUCCESS;
}

uint32_t log_callback_logview(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments)
{
    SEGGER_RTT_printf(0, "%u;%u;%s;%u;", dbg_level, timestamp, p_filename, line);
    SEGGER_RTT_vprintf(0, format, &arguments);
    SEGGER_RTT_Write(0, "$", 1);
    return NRF_SUCCESS;

}
#elif NRF51
uint32_t log_callback_rtt(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t log_callback_logview(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

#endif

#if HOST /* For unit tests and host builds */
uint32_t log_callback_stdout(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments)
{
    printf("<t: %10u>, %s, %4d, ",timestamp, p_filename, line);
    vprintf(format, arguments);
    return NRF_SUCCESS;
}
#endif

uint32_t log_init(uint32_t mask, uint32_t level, log_callback_t callback)
{
    g_log_debug_msk = mask;
    g_log_debug_level = level;

    m_log_callback = callback;
    return NRF_SUCCESS;
}

void log_set_callback(log_callback_t callback)
{
    m_log_callback = callback;
}

uint32_t log_printf(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, ...)
{
    va_list arguments;
    va_start(arguments, format);
    uint32_t status = log_vprintf(dbg_level, p_filename, line, timestamp, format, arguments);
    va_end(arguments);
    return status;
}

uint32_t log_vprintf(uint32_t dbg_level, const char * p_filename, uint16_t line,
    uint32_t timestamp, const char * format, va_list arguments)
{
    uint32_t status = NRF_SUCCESS;
    if(m_log_callback != NULL)
    {
        status = m_log_callback(dbg_level, p_filename, line, timestamp, format, arguments);
    }

    return status;
}

#endif
