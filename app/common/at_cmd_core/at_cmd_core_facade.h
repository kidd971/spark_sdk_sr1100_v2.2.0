/** @file  at_cmd_core_facade.h
 *  @brief Hardware interface required by the AT command core module.
 *
 *  Each application backend must implement these four functions.
 *  The expansion UART (USART2, PA2=TX / PA3=RX) is used for AT communication
 *  with an external MCU; it is independent from the STLink debug log UART.
 */
#ifndef AT_CMD_CORE_FACADE_H_
#define AT_CMD_CORE_FACADE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initialize the expansion UART (USART2, PA2=TX / PA3=RX).
 *
 *  @param[in] baud_rate  Baud rate (e.g. 115200).
 */
void facade_expansion_uart_init(uint32_t baud_rate);

/** @brief Transmit a null-terminated string over the expansion UART (blocking).
 *
 *  @param[in] string  Null-terminated string to transmit.
 */
void facade_expansion_uart_write(char *string);

/** @brief Read one byte from the expansion UART RX FIFO (non-blocking).
 *
 *  @return The received byte (1-255), or 0 if the FIFO is empty.
 */
uint8_t facade_expansion_uart_read_byte(void);

/** @brief Get the current system tick in milliseconds.
 *
 *  @return Current tick value in milliseconds.
 */
uint32_t facade_get_tick_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_CORE_FACADE_H_ */
