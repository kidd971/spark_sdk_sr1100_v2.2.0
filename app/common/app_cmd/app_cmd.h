/** @file  app_cmd.h
 *  @brief Application-level data channel command protocol.
 *
 *  Defines the command types and payload structure transmitted over the UWB
 *  data channel (tx_data_conn / rx_data_conn) between DG and HS STM32.
 *
 *  All commands use the same 2-byte app_cmd_t payload:
 *    - cmd_type : identifies the command (app_cmd_type_t)
 *    - value    : command-specific value (0 if unused)
 *
 *  Direction:
 *    DG → HS : CMD_VOL, CMD_PLAY, CMD_STOP, CMD_NEXT_TRACK, CMD_PRE_TRACK
 *    HS → DG : CMD_BATTERY, CMD_VOL (local change echo), CMD_PLAY (button event)
 *    Bidirectional: CMD_VOL (DG sets absolute; HS echoes local change back to DG)
 *                   CMD_PLAY (DG triggers; HS button reports back to DG)
 */
#ifndef APP_CMD_H_
#define APP_CMD_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Application command types transmitted over the UWB data channel. */
typedef enum {
    CMD_VOL        = 0x01, /*!< Set volume.          value: 0–100            (DG → HS) */
    CMD_PLAY       = 0x02, /*!< Play / Pause toggle. value: 0 (unused)       (DG → HS) */
    CMD_STOP       = 0x03, /*!< Stop playback.       value: 0 (unused)       (DG → HS) */
    CMD_NEXT_TRACK = 0x04, /*!< Skip to next track.  value: 0 (unused)       (DG → HS) */
    CMD_PRE_TRACK  = 0x05, /*!< Return to prev track.value: 0 (unused)       (DG → HS) */
    CMD_BATTERY    = 0x06, /*!< Battery level report. value: 0–100 (percent) (HS → DG) */
} app_cmd_type_t;

/** @brief Payload structure for application commands over the UWB data channel. */
typedef struct {
    uint8_t cmd_type; /*!< Command type (app_cmd_type_t). */
    uint8_t value;    /*!< Command value; 0 if unused.    */
} app_cmd_t;

#ifdef __cplusplus
}
#endif

#endif /* APP_CMD_H_ */
