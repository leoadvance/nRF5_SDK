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

#include "nrf_cli.h"
#include "nrf_cli_coap.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "thread_benchmark.h"
#include "thread_utils.h"
#include "app_scheduler.h"
#include "ble_advertiser.h"
#include "cli.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <openthread/ip6.h>
#include <openthread/openthread.h>

#define REMOTE_CMD_BUFFER_SIZE 128
#define IPV6_STRING_SIZE       (41 * sizeof(char))

static const otIp6Address m_unspecified_ipv6 =
{
    .mFields =
    {
        .m8 = {0}
    }
};

nrf_cli_t const * mp_cli_print;

static void address_to_string(const otIp6Address *addr, char * ipstr)
{
    UNUSED_VARIABLE(snprintf(ipstr, IPV6_STRING_SIZE, "%x:%x:%x:%x:%x:%x:%x:%x",
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 0)),
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 1)),
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 2)),
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 3)),
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 4)),
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 5)),
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 6)),
                    uint16_big_decode((uint8_t *)(addr->mFields.m16 + 7))));
}

static void print_test_results(thread_benchmark_result_t * p_results)
{
        float     throughput          = p_results->throughput;
        uint32_t  throughput_int      = (uint32_t)throughput;
        uint32_t  throughput_residuum = (uint32_t)((throughput - throughput_int) * 1000.0f);

        float     per          = p_results->per;
        uint32_t  per_int      = (uint32_t)per;
        uint32_t  per_residuum = (uint32_t)((per - per_int) * 1000.0f);

        thread_benchmark_status_t * p_status = thread_benchmark_status_get();

        nrf_cli_fprintf(mp_cli_print, NRF_CLI_INFO,
                        "\n\t=== Test Finished ===\n \
                        \rTest duration: %lums\n \
                        \rUDP packets received: %lu\n \
                        \rUDP payload received: %luB\n \
                        \rAcknowledgements lost: %lu\n \
                        \rTotal frames received: %lu\n \
                        \rIncorrect frames received: %lu\n\n \
                        \rThroughput: %lu.%luB/s\n \
                        \rPER: %lu.%lu%%\r\n",
                        p_results->duration,
                        p_results->rx_counters.packets_received,
                        p_results->rx_counters.bytes_received,
                        p_status->acks_lost,
                        p_results->rx_counters.rx_total,
                        p_results->rx_counters.rx_error,
                        throughput_int,
                        throughput_residuum,
                        per_int,
                        per_residuum);
}

static void print_discovered_peers(void * p_event_data, uint16_t event_size)
{
    const thread_benchmark_peer_information_t * p_peers = thread_benchmark_peer_table_get();

    char ipv6[IPV6_STRING_SIZE];
    uint32_t peer_device_id_lo;
    uint32_t peer_device_id_hi;

    nrf_cli_fprintf(mp_cli_print,
                    NRF_CLI_INFO,
                    "\r\n# ||    Device ID   || IPv6 Mesh-Local Endpoint ID\r\n");

    for (uint16_t i = 0; i < p_peers->current_peer_number; i++)
    {
        memset(ipv6, '\0', IPV6_STRING_SIZE);
        address_to_string(&p_peers->peer_table[i].peer_address, ipv6);

        peer_device_id_lo = (p_peers->peer_table[i].device_id & 0xFFFFFFFF);
        peer_device_id_hi = ((p_peers->peer_table[i].device_id >> 32) & 0xFFFFFFFF);

        nrf_cli_fprintf(mp_cli_print,
                        NRF_CLI_INFO,
                        "%d:  %08x%08x  %s\r\n",
                        i,
                        peer_device_id_hi,
                        peer_device_id_lo,
                        ipv6);
    }
}

static const char * configuration_mode_get(void)
{
    thread_benchmark_configuration_t * test_configuration = thread_benchmark_configuration_get();

    switch (test_configuration->mode)
    {
        case THREAD_BENCHMARK_MODE_UNIDIRECTIONAL:
        return "Unidirectional";

        case THREAD_BENCHMARK_MODE_ECHO:
        return "Echo";

        case THREAD_BENCHMARK_MODE_ACK:
        return "Ack";

        default:
        return "Unknown mode";
    }
}

static const char * thread_role_get(void)
{
    switch (otThreadGetDeviceRole(thread_ot_instance_get()))
    {
        case OT_DEVICE_ROLE_DISABLED:
        return "Disabled";

        case OT_DEVICE_ROLE_DETACHED:
        return "Detached";

        case OT_DEVICE_ROLE_CHILD:
        return "Child";

        case OT_DEVICE_ROLE_ROUTER:
        return "Router";

        case OT_DEVICE_ROLE_LEADER:
        return "Leader";

        default:
        return "Unknown role";
    }
}

static void cmd_config_get(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    char ipv6[IPV6_STRING_SIZE];

    uint64_t device_id    = thread_benchmark_local_device_id_get();
    uint32_t device_id_lo = device_id & 0xFFFFFFFF;
    uint32_t device_id_hi = (device_id >> 32) & 0xFFFFFFFF;

    memset(ipv6, '\0', IPV6_STRING_SIZE);
    address_to_string(otThreadGetMeshLocalEid(thread_ot_instance_get()), ipv6);

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "\n\t=== Local node information ===\r\n");
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Device ID:   %08x%08x\r\n", device_id_hi, device_id_lo);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "IPv6 ML-EID: %s\r\n", ipv6);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Thread Role: %s\r\n", thread_role_get());

    thread_benchmark_configuration_t * p_test_conf = thread_benchmark_configuration_get();

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "\n\t=== Test settings ===\r\n");
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Mode: %s\r\n", configuration_mode_get());
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "ACK Timeout: %d [ms]\r\n", p_test_conf->ack_timeout);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Packet count: %d\r\n", p_test_conf->count);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Payload size [B]: %d\r\n", p_test_conf->length);

    const thread_benchmark_peer_entry_t * p_peer = thread_benchmark_peer_selected_get();
    if (p_peer)
    {
        device_id_lo = p_peer->device_id & 0xFFFFFFFF;
        device_id_hi = (p_peer->device_id >> 32) & 0xFFFFFFFF;

        address_to_string(&p_peer->peer_address, ipv6);
    }
    else
    {
        device_id_lo = 0;
        device_id_hi = 0;

        address_to_string(&m_unspecified_ipv6, ipv6);
    }

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "\n\t=== Peer information ===\r\n");
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Device ID:   %08x%08x\r\n", device_id_hi, device_id_lo);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "IPv6 ML-EID: %s\r\n", ipv6);

}

static void default_cmd(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
    }
    else
    {
        nrf_cli_fprintf(p_cli,
                        NRF_CLI_ERROR,
                        "%s:%s%s\r\n",
                        argv[0],
                        " unknown parameter: ",
                        argv[1]);
    }
}

// Forward declaration of the handler function.
static void benchmark_evt_handler(thread_benchmark_evt_t * p_evt);

static void cmd_discover_peers(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    // Remeber cli used to start the test so results can be printed on the same interface.
    mp_cli_print = p_cli;

    thread_benchmark_test_init(benchmark_evt_handler);
    otError error = thread_benchmark_peer_discover();

    if (error != OT_ERROR_NONE)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Failed to sent discovery message\r\n");
    }
}

static void cmd_display_peers(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    char     ipv6[IPV6_STRING_SIZE];
    uint32_t peer_device_id_lo;
    uint32_t peer_device_id_hi;

    const thread_benchmark_peer_information_t * peer_information = thread_benchmark_peer_table_get();

    nrf_cli_fprintf(p_cli,
                    NRF_CLI_INFO,
                    "# ||    Device ID   || IPv6 Mesh-Local Endpoint ID\r\n");

    for (uint16_t i = 0; i < peer_information->current_peer_number; i++)
    {
        memset(ipv6, '\0', IPV6_STRING_SIZE);
        address_to_string(&peer_information->peer_table[i].peer_address, ipv6);

        peer_device_id_lo = (peer_information->peer_table[i].device_id & 0xFFFFFFFF);
        peer_device_id_hi = ((peer_information->peer_table[i].device_id >> 32) & 0xFFFFFFFF);

        nrf_cli_fprintf(p_cli,
                        NRF_CLI_INFO,
                        "%d:  %08x%08x  %s\r\n",
                        i,
                        peer_device_id_hi,
                        peer_device_id_lo,
                        ipv6);
    }
}

static void cmd_peer_select(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    if (argc < 2)
    {
       uint16_t selected_peer = thread_benchmark_peer_selected_id_get();

       nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "%d\r\n", selected_peer);
    }
    else if (argc == 2)
    {
        thread_benchmark_peer_select(atoi(argv[1]));
    }
    else
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR,"Too many arguments\r\n");
    }
}

static void cmd_test_start(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    if (!thread_benchmark_peer_selected_get())
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "No peer selected, please run:\r\n\t test peer discover \r\nto find peers\r\n");
        return;
    }

    // Remeber cli used to start the test so results can be printed on the same interface.
    mp_cli_print = p_cli;

    thread_benchmark_test_init(benchmark_evt_handler);

    otError error = thread_benchmark_peer_ctrl_request_send(TEST_START_REQUEST);

    if (error != OT_ERROR_NONE)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Failed to send test start control request\r\n");
        return;
    }
}

static void cmd_test_stop(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    thread_benchmark_status_t * p_status = thread_benchmark_status_get();

    p_status->test_in_progress = false;

    thread_benchmark_test_duration_calculate();

    otError error = thread_benchmark_peer_ctrl_request_send(TEST_STOP_REQUEST);

    if (error != OT_ERROR_NONE)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Failed to send test stop control request\r\n");
        return;
    }
}

static void cmd_test_results(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    otError error = thread_benchmark_results_request_send();

    if (error != OT_ERROR_NONE)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Failed to send test results request\r\n");
        return;
    }
}

static void cmd_mode_display(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO,"%s\r\n\n", configuration_mode_get());
    default_cmd(p_cli, argc, argv);
}

static void cmd_mode_unidirectional_set(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    thread_benchmark_configuration_t * p_test_configuration = thread_benchmark_configuration_get();
    p_test_configuration->mode                              = THREAD_BENCHMARK_MODE_UNIDIRECTIONAL;
}

static void cmd_mode_echo_set(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    thread_benchmark_configuration_t * p_test_configuration = thread_benchmark_configuration_get();
    p_test_configuration->mode                              = THREAD_BENCHMARK_MODE_ECHO;
}

static void cmd_mode_ack_set(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    thread_benchmark_configuration_t * p_test_configuration = thread_benchmark_configuration_get();
    p_test_configuration->mode                              = THREAD_BENCHMARK_MODE_ACK;
}

static void cmd_ack_timeout_set(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    thread_benchmark_configuration_t * p_test_configuration = thread_benchmark_configuration_get();

    if (argc < 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_INFO,"%d\r\n", p_test_configuration->ack_timeout);
    }
    else if (argc == 2)
    {
        p_test_configuration->ack_timeout = atoi(argv[1]);
    }
    else
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR,"Too many arguments\r\n");
    }
}

static void cmd_packet_count_set(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    thread_benchmark_configuration_t * p_test_configuration = thread_benchmark_configuration_get();

    if (argc < 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_INFO,"%d\r\n", p_test_configuration->count);
    }

    else if (argc == 2)
    {
        p_test_configuration->count = atoi(argv[1]);
    }

    else
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR,"Too many arguments\r\n");
    }
}

static void cmd_set_packet_length(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    thread_benchmark_configuration_t * p_test_configuration = thread_benchmark_configuration_get();

    if (argc < 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_INFO,"%d\r\n", p_test_configuration->length);
    }
    else if (argc == 2)
    {
        p_test_configuration->length = atoi(argv[1]);
    }
    else
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Too many arguments\r\n");
    }
}

static void display_ble_adv_info(nrf_cli_t const * p_cli)
{
    bool is_started        = ble_advertiser_is_started();
    uint32_t curr_interval = ble_advertiser_interval_get_current();
    uint32_t req_interval  = ble_advertiser_interval_get_requested();
    size_t data_len        = ble_advertiser_adv_data_len_get();

    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "\n\t=== Bluetooth advertising configuration ===\r\n");
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Started:                        %s\r\n", is_started ? "true" : "false");
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Current advertising interval:   %u ms\r\n", curr_interval);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Requested advertising interval: %u ms\r\n", req_interval);
    nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Advertising data length:        %d bytes\r\n", data_len);
}

static void cmd_ble_adv_start(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    ble_advertiser_start();
}

static void cmd_ble_adv_stop(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    ble_advertiser_stop();
}

static void cmd_ble_adv_interval(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    if (argc < 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Current:   %d\r\n", ble_advertiser_interval_get_current());
        nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "Requested: %d\r\n", ble_advertiser_interval_get_requested());
    }
    else if (argc == 2)
    {
        ble_advertiser_interval_set(atoi(argv[1]));
    }
    else
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Too many arguments\r\n");
    }
}

static void cmd_ble_adv_data_len(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    if (argc < 2)
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_INFO, "%d\r\n", ble_advertiser_adv_data_len_get());
    }
    else if (argc == 2)
    {
        if (!ble_advertiser_adv_data_len_set(atoi(argv[1])))
        {
            nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Data len can be in range 0 to %d, excluding 1\r\n", ble_advertiser_adv_data_len_max());
        }
    }
    else
    {
        nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Too many arguments\r\n");
    }
}

static void cmd_ble_adv_info(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    display_ble_adv_info(p_cli);
}

static void cmd_ble_info(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    display_ble_adv_info(p_cli);
}

static void cmd_remote(nrf_cli_t const * p_cli, size_t argc, char ** argv)
{
    nrf_cli_t const * p_peer_cli;
    char              rx_buffer[REMOTE_CMD_BUFFER_SIZE];
    char            * buf_ptr = rx_buffer;
	size_t            buf_len = REMOTE_CMD_BUFFER_SIZE;
	size_t            arg_len;

    if (argc < 3)
    {
       nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Required arguments not provided\r\n");
       return;
    }
    else
    {
        p_peer_cli = cli_coap_get(atoi(argv[1]));

        if (p_peer_cli == NULL)
        {
            nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "Remote number out of range\r\n");
            return;
        }
    }

    cli_coap_response_cli_set(p_cli);

	for (size_t i = 2; i < argc; i++) {
		arg_len = snprintf(buf_ptr, buf_len, "%s ", argv[i]);

		if (arg_len >= buf_len) {
			NRF_LOG_ERROR("Remote command buffer full");
			return;
		}

        buf_len -= arg_len;
        buf_ptr += arg_len;
	}

    // As we have spare space char at the end, replace it with \n char
    buf_ptr[-1] = '\n';

    cli_coap_send_cmd_to_peer(p_peer_cli, rx_buffer, REMOTE_CMD_BUFFER_SIZE - buf_len);
}

static void benchmark_evt_handler(thread_benchmark_evt_t * p_evt)
{
    uint16_t peer_count;
    uint32_t err_code;

    switch (p_evt->evt)
    {
        case THREAD_BENCHMARK_TEST_COMPLETED:
            NRF_LOG_INFO("Test completed");
            print_test_results(p_evt->context.p_result);
            break;

        case THREAD_BENCHMARK_TEST_STARTED:
            ASSERT(p_evt->context.error == OT_ERROR_NONE);
            NRF_LOG_INFO("Test started");
            break;

        case THREAD_BENCHMARK_TEST_STOPPED:
            ASSERT(p_evt->context.error == OT_ERROR_NONE);
            NRF_LOG_INFO("Test stopped");
            break;

        case THREAD_BENCHMARK_DISCOVERY_COMPLETED:
            peer_count = p_evt->context.p_peer_information->current_peer_number;

            if (peer_count)
            {
                thread_benchmark_peer_select(peer_count - 1);
            }

            NRF_LOG_INFO("Discovery completed, found %d peers", peer_count);

            for (size_t i = 0; i < CLI_COAP_INSTANCES && i < peer_count; i++)
            {
                cli_coap_peer_set(cli_coap_get(i), &thread_benchmark_peer_table_get()->peer_table[i].peer_address);
            }

            err_code = app_sched_event_put(NULL, 0, print_discovered_peers);
            ASSERT(err_code == NRF_SUCCESS);
            break;

        default:
            NRF_LOG_ERROR("Unknown thread_benchmark_evt");
            break;
    };
}

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_test_configure_mode_cmds)
{
    NRF_CLI_CMD(ack,            NULL, "Peer replies with a short acknowledgement", cmd_mode_ack_set),
    NRF_CLI_CMD(echo,           NULL, "Peer replies with the same data",           cmd_mode_echo_set),
    NRF_CLI_CMD(unidirectional, NULL, "Transmission in the single direction",      cmd_mode_unidirectional_set),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_test_configure_peer_cmds)
{
    NRF_CLI_CMD(discover, NULL,                 "Discover available peers", cmd_discover_peers),
    NRF_CLI_CMD(list,     NULL,                 "Display found peers",      cmd_display_peers),
    NRF_CLI_CMD(results,  NULL,                 "Display results",          cmd_test_results),
    NRF_CLI_CMD(select,   NULL,                 "Select peer from a list",  cmd_peer_select),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_test_configure_cmds)
{
    NRF_CLI_CMD(ack-timeout,   NULL,                        "Set time after we stop waiting for the acknowledgement from the peer in miliseconds", cmd_ack_timeout_set),
    NRF_CLI_CMD(count,         NULL,                        "Set number of packets to be sent",                                                    cmd_packet_count_set),
    NRF_CLI_CMD(lenght,        NULL,                        "Set UDP payload length in bytes",                                                     cmd_set_packet_length),
    NRF_CLI_CMD(mode,          &m_test_configure_mode_cmds, "Set test type",                                                                       cmd_mode_display),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_test_cmds)
{
    NRF_CLI_CMD(configure, &m_test_configure_cmds,      "Configure the test",                default_cmd),
    NRF_CLI_CMD(info,      NULL,                        "Print current configuration",       cmd_config_get),
    NRF_CLI_CMD(peer,      &m_test_configure_peer_cmds, "Configure the peer receiving data", default_cmd),
    NRF_CLI_CMD(start,     NULL,                        "Start the test",                    cmd_test_start),
    NRF_CLI_CMD(stop,      NULL,                        "Stop the test",                     cmd_test_stop),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CMD_REGISTER(test, &m_test_cmds, "Benchmark commands", default_cmd);

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_test_configure_bluetooth_advertising_cmds)
{
    NRF_CLI_CMD(info,     NULL, "Display summary of Bluetooth advertising configuration", cmd_ble_adv_info),
    NRF_CLI_CMD(interval, NULL, "Set Bluetooth advertising interval",                     cmd_ble_adv_interval),
    NRF_CLI_CMD(size,     NULL, "Set Bluetooth advertising data length",                  cmd_ble_adv_data_len),
    NRF_CLI_CMD(start,    NULL, "Start Bluetooth advertising",                            cmd_ble_adv_start),
    NRF_CLI_CMD(stop,     NULL, "Stop Bluetooth advertising",                             cmd_ble_adv_stop),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_test_configure_bluetooth_cmds)
{
    NRF_CLI_CMD(adv,  &m_test_configure_bluetooth_advertising_cmds, "Configure Bluetooth advertising",            default_cmd),
    NRF_CLI_CMD(info, NULL,                                         "Display summary of Bluetooth configuration", cmd_ble_info),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CMD_REGISTER(ble, &m_test_configure_bluetooth_cmds, "Configure Bluetooth options", default_cmd);

NRF_CLI_CMD_REGISTER(remote, NULL, "Send command to remote peer", cmd_remote);
