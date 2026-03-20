/** @file  at_cmd_core.c
 *  @brief Common AT command core implementation.
 */

/* INCLUDES *******************************************************************/
#include "at_cmd_core.h"
#include "at_cmd_core_facade.h"
#include "at_module.h"
#include "swc_api.h"
#include "swc_error.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static bool handler_ver(const char *args, char *resp, uint16_t resp_size);
static bool handler_help(const char *args, char *resp, uint16_t resp_size);
static bool handler_ping(const char *args, char *resp, uint16_t resp_size);
static bool handler_module_info(const char *args, char *resp, uint16_t resp_size);
static bool handler_uwb_conn_status(const char *args, char *resp, uint16_t resp_size);
static bool handler_uwb_pair(const char *args, char *resp, uint16_t resp_size);
static bool handler_fw_version(const char *args, char *resp, uint16_t resp_size);

/* PRIVATE VARIABLES **********************************************************/
static uint8_t              s_device_address   = 0xFF;
static at_uwb_conn_status_t s_uwb_conn_status  = AT_UWB_CONN_STATUS_STANDBY;
static void               (*s_pair_cb)(void)    = NULL;
static bool               (*s_link_status_cb)(void) = NULL;
static bool                 s_pair_requested   = false;

/* PUBLIC FUNCTIONS ***********************************************************/
void at_cmd_core_init(void)
{
    facade_expansion_uart_init(AT_CMD_CORE_BAUD_RATE);

    at_module_init(facade_expansion_uart_write,
                   facade_expansion_uart_read_byte,
                   facade_get_tick_ms);

    at_server_register("VER",    handler_ver);
    at_server_register("HELP",   handler_help);
    at_server_register("PING",        handler_ping);
    at_server_register("MODULE_INFO",    handler_module_info);
    at_server_register("UWB_CONN_STATUS", handler_uwb_conn_status);
    at_server_register("UWB_PAIR",        handler_uwb_pair);
    at_server_register("FW_VERSION",      handler_fw_version);
}

void at_cmd_core_register_link_status_cb(bool (*cb)(void))
{
    s_link_status_cb = cb;
}

void at_cmd_core_notify_uwb_ready(void)
{
    facade_expansion_uart_write("+EVENT: UWB_READY\r\n");
}

void at_cmd_core_register_pair_cb(void (*cb)(void))
{
    s_pair_cb = cb;
}

void at_cmd_core_set_uwb_conn_status(at_uwb_conn_status_t status)
{
    s_uwb_conn_status = status;
}

void at_cmd_core_set_device_address(uint8_t addr)
{
    s_device_address = addr;
}

void at_cmd_core_process(void)
{
    at_module_process();

    /* Invoke pair callback deferred — after at_module_process() has sent OK. */
    if (s_pair_requested) {
        s_pair_requested = false;
        if (s_pair_cb != NULL) {
            s_pair_cb();
        }
    }

    if (s_link_status_cb == NULL || s_uwb_conn_status == AT_UWB_CONN_STATUS_PAIRING) {
        return;
    }

    at_uwb_conn_status_t new_status = s_link_status_cb()
                                      ? AT_UWB_CONN_STATUS_CONNECTED
                                      : AT_UWB_CONN_STATUS_STANDBY;

    if (new_status != s_uwb_conn_status) {
        s_uwb_conn_status = new_status;
        if (new_status == AT_UWB_CONN_STATUS_CONNECTED) {
            facade_expansion_uart_write("+EVENT: UWB_CONNECTED\r\n");
        } else {
            facade_expansion_uart_write("+EVENT: UWB_DISCONNECTED\r\n");
        }
    }
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief AT+VER — return SDK version string. */
static bool handler_ver(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    snprintf(resp, resp_size, "+VER: SPARK SDK SR1100 v2.2.0");
    return true;
}

/** @brief AT+FW_VERSION? — return firmware version string. */
static bool handler_fw_version(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    snprintf(resp, resp_size, "+FW_VERSION: v2.2.0");
    return true;
}

/** @brief AT+PING — connectivity check, always responds OK. */
static bool handler_ping(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    snprintf(resp, resp_size, "OK");
    return true;
}

/** @brief AT+MODULE_INFO? — HW model, FW version, chip version, IC serial number, device address. */
static bool handler_module_info(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    swc_error_t err = SWC_ERR_NONE;

    uint8_t  hw_model   = swc_node_get_radio_product_model(&err);
    uint8_t  chip_ver   = swc_node_get_radio_product_version(&err);
    uint64_t serial     = swc_node_get_radio_serial_number(&err);

    snprintf(resp, resp_size,
             "+MODULE_INFO: HW=%u,FW=v2.2.0,Chip=%u,SN=%08lX%08lX,Addr=0x%02X",
             hw_model,
             chip_ver,
             (unsigned long)(serial >> 32),
             (unsigned long)(serial & 0xFFFFFFFF),
             s_device_address);
    return true;
}

/** @brief AT+UWB_CONN_STATUS? — current UWB connection status (0=Standby, 1=Pairing, 2=Connected). */
static bool handler_uwb_conn_status(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    snprintf(resp, resp_size, "+UWB_CONN_STATUS: %d", (int)s_uwb_conn_status);
    return true;
}

/** @brief AT+UWB_PAIR — trigger UWB pairing via registered callback. */
static bool handler_uwb_pair(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    s_pair_requested = true;
    snprintf(resp, resp_size, "OK");
    return true;
}

/** @brief AT+HELP — list all registered AT commands. */
static bool handler_help(const char *args, char *resp, uint16_t resp_size)
{
    (void)args;
    (void)resp_size;

    /* Query commands end with '?'; action commands do not. */
    static const char * const cmds[] = {
        "  AT+HELP\r\n",
        "  AT+PING\r\n",
        "  AT+VER?\r\n",
        "  AT+FW_VERSION?\r\n",
        "  AT+MODULE_INFO?\r\n",
        "  AT+UWB_CONN_STATUS?\r\n",
        "  AT+UWB_PAIR\r\n",
    };

    facade_expansion_uart_write("+HELP:\r\n");
    for (uint8_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        facade_expansion_uart_write((char *)cmds[i]);
    }

    resp[0] = '\0'; /* at_module will append OK\r\n */
    return true;
}
