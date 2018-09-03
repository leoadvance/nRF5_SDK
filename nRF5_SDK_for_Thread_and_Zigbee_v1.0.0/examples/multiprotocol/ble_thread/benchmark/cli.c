#include "cli.h"

#include "bsp.h"
#include "nrf_cli.h"
#include "nrf_cli_rtt.h"
#include "nrf_cli_uart.h"
#include "nrf_cli_coap.h"

#include "thread_utils.h"

/**@brief CLI configuration. */
#define CLI_UART_INSTANCE_ID        1
#define CLI_UART_TX_BUF_SIZE        64
#define CLI_UART_RX_BUF_SIZE        16
#define CLI_COAP_RX_BUF_SIZE        128

/**@brief CLI instance. */
#define CLI_LOG_QUEUE_SIZE          4

NRF_CLI_UART_DEF(m_cli_uart_transport,
                 CLI_UART_INSTANCE_ID,
                 CLI_UART_TX_BUF_SIZE,
                 CLI_UART_RX_BUF_SIZE);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            CLI_LOG_QUEUE_SIZE);

NRF_CLI_RTT_DEF(m_cli_rtt_transport);
NRF_CLI_DEF(m_cli_rtt,
            "rtt_cli:~$ ",
            &m_cli_rtt_transport.transport,
            '\n',
            CLI_LOG_QUEUE_SIZE);

NRF_CLI_COAP_DEF_MULTI(CLI_COAP_INSTANCES,
                       m_cli_coap_transport,
                       CLI_COAP_RX_BUF_SIZE);
NRF_CLI_DEF_MULTI(CLI_COAP_INSTANCES,
                  m_cli_coap,
                  "",
                  m_cli_coap_transport,
                  '\n',
                  CLI_LOG_QUEUE_SIZE);

/**@brief Function for initializing the CLI.
 *
 * Serial connection and RTT console are supported.
 */
void cli_init(void)
{
    ret_code_t err_code = nrf_cli_init(&m_cli_rtt, NULL, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(err_code);

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd               = TX_PIN_NUMBER;
    uart_config.pselrxd               = RX_PIN_NUMBER;
    uart_config.hwfc                  = NRF_UART_HWFC_DISABLED;

    err_code = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(err_code);
}

void cli_coap_init(void)
{
    for (size_t i = 0; i < CLI_COAP_INSTANCES; i++)
    {
        ret_code_t err_code = nrf_cli_init(m_cli_coap[i], thread_ot_instance_get(), true, false, NRF_LOG_SEVERITY_INFO);
        APP_ERROR_CHECK(err_code);

        m_cli_coap[i]->p_ctx->internal.flag.echo = true;
    }
}

/**@brief Function for starting the CLI.
 */
void cli_start(void)
{
    ret_code_t err_code = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_cli_start(&m_cli_rtt);
    APP_ERROR_CHECK(err_code);
}

void cli_coap_start(void)
{
    for (size_t i = 0; i < CLI_COAP_INSTANCES; i++)
    {
        ret_code_t err_code = nrf_cli_start(m_cli_coap[i]);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief  Function for processing CLI operations.
 */
void cli_process(void)
{
    nrf_cli_process(&m_cli_uart);
    nrf_cli_process(&m_cli_rtt);

    for(size_t i = 0; i < CLI_COAP_INSTANCES; i++)
    {
        nrf_cli_process(m_cli_coap[i]);
    }
}

nrf_cli_t const * cli_coap_get(size_t idx)
{
    if (idx < CLI_COAP_INSTANCES)
    {
        return m_cli_coap[idx];
    }
    else
    {
        return NULL;
    }
}
