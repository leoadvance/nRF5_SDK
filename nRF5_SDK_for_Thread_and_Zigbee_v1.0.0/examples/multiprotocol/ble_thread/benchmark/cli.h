#ifndef CLI_H__
#define CLI_H__

#include "nrf_cli.h"

#define CLI_COAP_INSTANCES 5

/**@brief Function for initializing the CLI.
 *
 * Serial connection and RTT console are supported.
 */
void cli_init(void);

/**@brief Function for initializing the CoAP CLI.
 */
void cli_coap_init(void);

/**@brief Function for starting the CLI.
 */
void cli_start(void);

/**@brief Function for starting the CoAP CLI.
 */
void cli_coap_start(void);

/**@brief Function for processing CLI operations.
 */
void cli_process(void);

/**@brief Get CoAP CLI instance from index
 */
nrf_cli_t const * cli_coap_get(size_t idx);

#endif /* CLI_H__ */
