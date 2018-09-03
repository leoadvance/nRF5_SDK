/**
 * Copyright (c) 2017 - 2018, Nordic Semiconductor ASA
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
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "bsp.h"

#include "sdk_config.h"
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nordic_common.h"
#include "nrf_gpio.h"

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bsp_thread.h"
#include "thread_utils.h"
#include "thread_coap_utils.h"

#include "thread_benchmark.h"
#include "test_transmission_udp.h"

#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh.h"
#include "nrf_soc.h"

#include "cli.h"

#include <openthread/platform/platform-softdevice.h>

#define APP_BLE_OBSERVER_PRIO       1                               /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG        1                               /**< A tag identifying the SoftDevice BLE configuration. */

/**@brief Scheduler configuration. */
#define SCHED_MAX_EVENT_DATA_SIZE   sizeof(frame_information_t)
#define SCHED_QUEUE_SIZE            32

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
    APP_ERROR_CHECK(err_code);
}

/***************************************************************************************************
 * @section Callbacks
 **************************************************************************************************/

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d",
                     flags,
                     otThreadGetDeviceRole(p_context));
    }

}

/**@brief Function for handling SOC events.
 *
 * @param[in]   sys_evt     SoC stack event.
 * @param[in]   p_context   Unused.
 */
static void soc_evt_handler(uint32_t sys_evt, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    PlatformSoftdeviceSocEvtHandler(sys_evt);
}

/***************************************************************************************************
 * @section Init
 **************************************************************************************************/

/**@brief Function for initializing the Thread Board Support Package.
 */
static void thread_bsp_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Thread Stack.
 */
static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .role                  = RX_ON_WHEN_IDLE,
        .autocommissioning     = true,
        .poll_period           = 2500,
        .default_child_timeout = 10,
    };

    thread_init(&thread_configuration);
    thread_state_changed_callback_set(thread_state_changed_callback);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Power Driver.
 */
static void power_driver_init(void)
{
    ret_code_t ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for initializing the Clock Driver.
 */
static void clock_driver_init(void)
{
    ret_code_t ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
}

static void coap_handler_default(void                * p_context,
                                 otCoapHeader        * p_header,
                                 otMessage           * p_message,
                                 const otMessageInfo * p_message_info)
{
    UNUSED_VARIABLE(p_context);
    UNUSED_VARIABLE(p_header);
    UNUSED_VARIABLE(p_message);
    UNUSED_VARIABLE(p_message_info);

    NRF_LOG_INFO("Received CoAP message that does not match any request or resource");
}

/**@brief Function for initializing the CoAP.
 */
static void coap_init(void)
{
    otError error = otCoapStart(thread_ot_instance_get(), OT_DEFAULT_COAP_PORT);
    ASSERT(error == OT_ERROR_NONE);

    otCoapSetDefaultHandler(thread_ot_instance_get(), coap_handler_default, NULL);
}

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

    // Register a handler for SOC events.
    NRF_SDH_SOC_OBSERVER(m_soc_observer, NRF_SDH_SOC_STACK_OBSERVER_PRIO, soc_evt_handler, NULL);
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

int main(void)
{
    log_init();
    clock_driver_init();
    scheduler_init();
    power_driver_init();

    timer_init();

    ble_stack_init();

    cli_init();
    cli_start();

    // Initialize the Thread stack.
    thread_instance_init();
    coap_init();
    thread_benchmark_init();
    thread_bsp_init();

    cli_coap_init();
    cli_coap_start();

    for (;;)
    {
        cli_process();
        thread_benchmark_process();
        thread_process();
        app_sched_execute();

        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }
    }
}


/**
 * @}
 */
