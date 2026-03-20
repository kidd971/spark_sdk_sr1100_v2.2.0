/** @file  at_cmd_core.h
 *  @brief Common AT command core — shared across all applications.
 *
 *  Wraps at_module with the expansion UART backend and registers a set of
 *  built-in commands available in every application:
 *
 *    AT+VER         — firmware / SDK version string
 *    AT+STATUS      — system uptime in milliseconds
 *    AT+HELP        — list all registered AT commands
 *    AT+PING            — connectivity check, responds OK
 *    AT+MODULE_INFO?    — HW model, FW version, chip version, IC serial, device address
 *    AT+UWB_CONN_STATUS? — current UWB connection status (0=Standby,1=Pairing,2=Connected)
 *    AT+UWB_PAIR        — trigger UWB pairing (invokes registered pair callback)
 *
 *  Applications that need additional commands call at_server_register()
 *  directly after at_cmd_core_init().
 *
 *  Usage:
 *    1. Call at_cmd_core_init() once during application startup.
 *    2. Optionally register app-specific commands with at_server_register().
 *    3. Call at_cmd_core_process() every iteration of the main loop.
 */
#ifndef AT_CMD_CORE_H_
#define AT_CMD_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Baud rate used for the expansion UART AT channel. */
#define AT_CMD_CORE_BAUD_RATE  115200

/** @brief UWB connection status codes reported by AT+UWB_CONN_STATUS?. */
typedef enum {
    AT_UWB_CONN_STATUS_STANDBY   = 0, /*!< Idle, not yet started. */
    AT_UWB_CONN_STATUS_PAIRING   = 1, /*!< Pairing procedure in progress. */
    AT_UWB_CONN_STATUS_CONNECTED = 2, /*!< Link established and running. */
} at_uwb_conn_status_t;

/**
 * @brief Initialize the AT command core.
 *
 * Initializes the expansion UART, the AT module, and registers the built-in
 * commands (VER, STATUS, HELP).
 */
void at_cmd_core_init(void);

/**
 * @brief Drive the AT command state machine.
 *
 * Must be called every iteration of the application main loop.
 */
void at_cmd_core_process(void);

/**
 * @brief Update the UWB connection status reported by AT+UWB_CONN_STATUS?.
 *
 * Call this whenever the application state machine transitions state.
 * Default value before any call is AT_UWB_CONN_STATUS_STANDBY.
 *
 * @param[in] status  New connection status.
 */
void at_cmd_core_set_uwb_conn_status(at_uwb_conn_status_t status);

/**
 * @brief Set the local device address reported by AT+MODULE_INFO?.
 *
 * Call this after swc_setup() with the node's local_address.
 * If not called, address is reported as 0xFF.
 *
 * @param[in] addr  Local device address from swc_node_cfg_t.local_address.
 */
void at_cmd_core_set_device_address(uint8_t addr);

/**
 * @brief Send +EVENT: UWB_READY to the external MCU.
 *
 * Call once after SWC initialization is complete and the device is ready
 * to connect or accept pairing.
 */
void at_cmd_core_notify_uwb_ready(void);

/**
 * @brief Register a connection status getter polled by at_cmd_core_process().
 *
 * The callback returns true when the UWB link is up, false otherwise.
 * at_cmd_core_process() polls this every main-loop iteration and sends
 * +EVENT: UWB_CONNECTED or +EVENT: UWB_DISCONNECTED only when the status
 * changes. Polling is skipped while status is AT_UWB_CONN_STATUS_PAIRING.
 *
 * Typical usage (call once after app_init()):
 *   at_cmd_core_register_link_status_cb(my_get_link_status);
 *
 * @param[in] cb  Function returning true if link is up. May be NULL to unregister.
 */
void at_cmd_core_register_link_status_cb(bool (*cb)(void));

/**
 * @brief Register a callback invoked when AT+UWB_PAIR is received.
 *
 * The callback should initiate the application's pairing procedure.
 * If no callback is registered the command still returns OK but does nothing.
 *
 * @param[in] cb  Function to call on AT+UWB_PAIR. May be NULL to unregister.
 */
void at_cmd_core_register_pair_cb(void (*cb)(void));

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_CORE_H_ */
