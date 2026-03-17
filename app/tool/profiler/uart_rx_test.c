/** @file  uart_rx_test.c
 *  @brief UART RX sample - IRQ-based echo test.
 *
 *  Select the UART port by setting USE_EXPANSION_UART below:
 *
 *    0 -> STLink virtual COM port  (UART4,  PC10=TX / PC11=RX, AF8, 115200)
 *    1 -> Expansion connector      (USART2, PA2=TX  / PA3=RX,  AF7, configurable)
 *
 *  The profiler debug log (facade_log_*) is NOT affected by this switch.
 *
 *  Test procedure:
 *    1. Set USE_EXPANSION_UART to 0 or 1 and rebuild.
 *    2. Open the corresponding COM port at UART_BAUD_RATE, 8N1.
 *    3. Type text and press Enter -> board echoes "ECHO: [text]".
 */

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "profiler_facade.h"

/* CONFIGURATION **************************************************************/
/** @brief Set to 0 for STLink UART4 (PC10/PC11), 1 for Expansion USART2 (PA2/PA3). */
#define USE_EXPANSION_UART  1

/** @brief Baud rate used when USE_EXPANSION_UART == 1. */
#define EXPANSION_BAUD_RATE 115200

/* UART ABSTRACTION ***********************************************************/
#if USE_EXPANSION_UART
    #define UART_INIT()       facade_expansion_uart_init(EXPANSION_BAUD_RATE)
    #define UART_WRITE(s)     facade_expansion_uart_write(s)
    #define UART_READ_BYTE()  facade_expansion_uart_read_byte()
    #define UART_PORT_NAME    "Expansion USART2 (PA2/PA3)"
#else
    #define UART_INIT()       facade_log_init()
    #define UART_WRITE(s)     facade_log_write(s)
    #define UART_READ_BYTE()  facade_log_read_byte()
    #define UART_PORT_NAME    "STLink UART4 (PC10/PC11)"
#endif

/* CONSTANTS ******************************************************************/
#define RX_LINE_MAX 256

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void rx_echo_loop(void);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    facade_board_init();
    UART_INIT();

    UART_WRITE("\r\n=== UART RX Test ===\r\n");
    UART_WRITE("Port : " UART_PORT_NAME "\r\n");
    UART_WRITE("Type a line and press Enter.\r\n\r\n");

    rx_echo_loop();

    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
static void rx_echo_loop(void)
{
    char     rx_line[RX_LINE_MAX];
    char     tx_line[RX_LINE_MAX + 16];
    uint16_t idx = 0;
    uint8_t  ch;

    while (1) {
        ch = UART_READ_BYTE();

        if (ch == 0) {
            continue;
        }

        /* Immediate local echo */
        char echo[2] = {(char)ch, '\0'};
        UART_WRITE(echo);

        if (ch == '\r' || ch == '\n') {
            if (idx == 0) {
                continue;
            }
            rx_line[idx] = '\0';
            int len = snprintf(tx_line, sizeof(tx_line),
                               "\r\nECHO: [%s]\r\n\r\n", rx_line);
            (void)len;
            UART_WRITE(tx_line);
            idx = 0;
        } else if (idx < RX_LINE_MAX - 1) {
            rx_line[idx++] = (char)ch;
        }
    }
}
