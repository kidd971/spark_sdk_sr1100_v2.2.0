/** @file  at_cmd_core.c
 *  @brief Common AT command core implementation.
 */

/* INCLUDES *******************************************************************/
#include "at_cmd_core.h"
#include "at_cmd_core_facade.h"
#include "at_module.h"
#include <stdint.h>
#include <stdio.h>

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static bool handler_ver(const char *args, char *resp, uint16_t resp_size);
static bool handler_status(const char *args, char *resp, uint16_t resp_size);
static bool handler_help(const char *args, char *resp, uint16_t resp_size);

/* PUBLIC FUNCTIONS ***********************************************************/
void at_cmd_core_init(void)
{
    facade_expansion_uart_init(AT_CMD_CORE_BAUD_RATE);

    at_module_init(facade_expansion_uart_write,
                   facade_expansion_uart_read_byte,
                   facade_get_tick_ms);

    at_server_register("VER",    handler_ver);
    at_server_register("STATUS", handler_status);
    at_server_register("HELP",   handler_help);
}

void at_cmd_core_process(void)
{
    at_module_process();
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief AT+VER — return SDK version string. */
static bool handler_ver(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    snprintf(resp, resp_size, "+VER: SPARK SDK SR1100 v2.2.0");
    return true;
}

/** @brief AT+STATUS — return system uptime in milliseconds. */
static bool handler_status(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    snprintf(resp, resp_size, "+STATUS: uptime=%lums", (unsigned long)facade_get_tick_ms());
    return true;
}

/** @brief AT+HELP — list all registered AT commands. */
static bool handler_help(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    char names[256];
    at_server_list_commands(names, sizeof(names));
    snprintf(resp, resp_size, "+HELP: AT+%s",
             names[0] != '\0' ? names : "(none)");
    return true;
}
