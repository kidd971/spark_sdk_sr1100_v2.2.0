/** @file  audio_unidirectional_coord.c
 *  @brief This application creates a unidirectional audio stream at 48kHz/24-bit depth from the I2S interface of the
 *         Coordinator to the I2S of the Node. It includes a fallback to 16 bits to ensure audio quality under varying
 *         conditions. Additionally, there is a bidirectional link for user data and link margin, which supports dynamic
 *         fallback updates.
 *
 *  @copyright Copyright (C) 2025 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES ******************************************************************/
#include <stdio.h>
#include "audio_unidirectional_facade.h"
#include "pairing_api.h"
#include "pairing_cfg.h"
#include "sac_api.h"
#include "sac_cfg.h"
#include "sac_endpoint_swc.h"
#include "sac_fallback.h"
#include "sac_fallback_gate.h"
#include "sac_hal_facade.h"
#include "sac_packing.h"
#include "sac_stats.h"
#include "swc_api.h"
#include "swc_cfg.h"
#include "swc_cfg_coord.h"
#include "swc_error.h"
#include "swc_stats.h"

/* CONSTANTS ******************************************************************/
/* Total memory needed for the Audio Core. */
#define SAC_MEM_POOL_SIZE 10000
/* Total memory needed for the Wireless Core. */
#define SWC_MEM_POOL_SIZE 10000
/* The data connection supports up to 16 bytes. */
#define MAX_DATA_PAYLOAD_SIZE 16
/* Length of the statistics array used for terminal display. */
#define STATS_ARRAY_LENGTH 3000
/* Period for data transmission timer in ms. */
#define DATA_TX_PERIOD_MS 10
/* Period for statistics print timer in ms. */
#define STATS_PRINT_PERIOD_MS 1000
/* Size of the buffer used to print errors. */
#define ERROR_MESSAGE_BUFFER_SIZE 50
/* Interval to print statistics in ms. */
#define PRINT_INTERVAL_MS 1000

/* **** Fallback **** */
/* During fallback mode transfers, audio samples are packed into 16 bits instead of 24 bits. */
#define FALLBACK_PAYLOAD_SIZE \
    SAC_CALCULATE_PAYLOAD_SIZE(MAIN_CHANNEL_SAMPLE_COUNT, MAIN_CHANNEL_CHANNEL_COUNT, SAC_16BITS)

/* TYPES **********************************************************************/
/** @brief Enumeration representing device pairing states.
 */
typedef enum device_pairing_state {
    /*! The device is unpaired with the Node. */
    DEVICE_UNPAIRED,
    /*! The device is paired with the Node. */
    DEVICE_PAIRED,
} device_pairing_state_t;

/** @brief Enumeration representing the connection priorities.
 */
typedef enum connection_priority {
    /*! Audio connection priority allows prioritizing audio transfers. */
    AUDIO_CONNECTION_PRIORITY = 0,
    /*! Data connection priority allows data transfers without compromising audio transfers. */
    DATA_CONNECTION_PRIORITY = 1,
} connection_priority_t;

/** @brief Data used for transmitting and receiving link margin and button state.
 */
typedef struct user_data {
    /*! A boolean indicating the button's state. */
    bool button_state;
    /*! The link margin to monitor link quality. */
    uint8_t link_margin;
} user_data_t;

/* PRIVATE GLOBALS ************************************************************/
/* **** Audio Core **** */
static uint8_t audio_memory_pool[SAC_MEM_POOL_SIZE];
/* Memory corruption sentinels */
static uint32_t audio_pool_guard_head = 0xDEADBEEF;
static uint32_t audio_pool_guard_tail = 0xA5A5A5A5;
static void coord_check_audio_pool_guards(const char *tag)
{
    if (audio_pool_guard_head != 0xDEADBEEF || audio_pool_guard_tail != 0xA5A5A5A5) {
        char msg[80];
        sprintf(msg, "CORRUPTION(coord)! %s head=%08lX tail=%08lX\n", tag,
                (unsigned long)audio_pool_guard_head,
                (unsigned long)audio_pool_guard_tail);
        facade_print_error_string(msg);
    }
}
static sac_pipeline_t *sac_pipeline;

/* **** Processing Stages **** */
static sac_fallback_instance_t sac_fallback_instance;
static sac_processing_t *sac_fallback_processing;
static sac_packing_instance_t audio_packing_instance;
static sac_processing_t *sac_packing_processing;
static sac_packing_instance_t audio_packing_fallback_instance;
static sac_processing_t *sac_packing_fallback_processing;

/* **** Endpoints **** */
static sac_endpoint_t *i2s_producer;
static ep_swc_instance_t swc_consumer_instance;
static sac_endpoint_t *swc_consumer;
/* Debug last TX sequence number updated via weak hook */
static volatile uint32_t coord_last_tx_seq;
void sac_debug_update_tx_seq(uint32_t seq) { coord_last_tx_seq = seq; }

/* Debug: capture first I2S samples sent by Coordinator */
volatile uint32_t coord_last_left_sample = 0;
volatile uint32_t coord_last_right_sample = 0;
volatile uint16_t coord_last_payload_size = 0;

/* Enable to inject a test tone into I2S producer (pre-packing) for debugging */
#ifndef COORD_INJECT_TEST_TONE
#define COORD_INJECT_TEST_TONE 0
#endif
static uint32_t coord_tone_phase = 0;

void sac_debug_update_i2s_rx_samples(const uint8_t *samples, uint16_t size)
{
    if (samples == NULL || size < 8) {
        return;
    }
    const uint32_t *p32 = (const uint32_t *)samples;
    coord_last_left_sample = p32[0];
    coord_last_right_sample = p32[1];
    coord_last_payload_size = size;
}

/* Optional strong hook: override I2S RX samples with a simple test pattern */
void sac_debug_override_i2s_rx_samples(uint8_t *samples, uint16_t size)
{
#if COORD_INJECT_TEST_TONE
    if (samples == NULL || size < 8) {
        return;
    }
    /* Samples are 32-bit per channel, interleaved LR at I2S input */
    uint32_t *p32 = (uint32_t *)samples;
    uint16_t frames = (uint16_t)(size / (2u * sizeof(uint32_t)));
    for (uint16_t i = 0; i < frames; i++) {
        uint32_t l = (coord_tone_phase & 0x00FFFFFFu);      /* 24-bit ramp */
        uint32_t r = ((~coord_tone_phase) & 0x00FFFFFFu);   /* inverted */
        p32[2u * i + 0u] = l;
        p32[2u * i + 1u] = r;
        coord_tone_phase += 0x000100u; /* step */
    }
#else
    (void)samples;
    (void)size;
#endif
}

/* Override at producer enqueue stage: ensures injection happens after DMA completes */
void sac_debug_override_producer_packet(uint8_t *packet, uint16_t size, bool encapsulated)
{
#if COORD_INJECT_TEST_TONE
    (void)encapsulated; /* I2S producer uses raw payload (non-encapsulated) */
    if (packet == NULL || size < 8) {
        return;
    }
    /* Inject 32-bit interleaved LR ramp to validate path */
    uint32_t *p32 = (uint32_t *)packet;
    uint16_t frames = (uint16_t)(size / (2u * sizeof(uint32_t)));
    for (uint16_t i = 0; i < frames; i++) {
        uint32_t l = (coord_tone_phase & 0x00FFFFFFu);
        uint32_t r = ((~coord_tone_phase) & 0x00FFFFFFu);
        p32[2u * i + 0u] = l;
        p32[2u * i + 1u] = r;
        coord_tone_phase += 0x000100u;
    }
#else
    (void)packet;
    (void)size;
    (void)encapsulated;
#endif
}

/* Debug: capture actual SWC TX packet (header + payload) just before OTA */
void sac_debug_update_swc_tx_packet(const uint8_t *packet, uint16_t size)
{
    if (packet == NULL || size < sizeof(sac_header_t)) {
        return;
    }
    const sac_header_t *hdr = (const sac_header_t *)packet;
    uint16_t total_needed = (uint16_t)(sizeof(sac_header_t) + hdr->payload_size);
    if (size < total_needed) {
        return;
    }
    const uint8_t *data = packet + sizeof(sac_header_t);
    coord_last_payload_size = hdr->payload_size;

    /* Extract first stereo frame from payload depending on fallback flag */
    if (hdr->fallback) {
        /* Fallback mode: 16-bit packed per sample per channel */
        if (hdr->payload_size < 4) {
            return;
        }
        const uint16_t *p16 = (const uint16_t *)data;
        coord_last_left_sample = (uint32_t)p16[0];
        coord_last_right_sample = (uint32_t)p16[1];
    } else {
        /* Normal mode: 24-bit packed samples (LSB-aligned in 3 bytes) */
        if (hdr->payload_size < 6) {
            return;
        }
        uint32_t l = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
        uint32_t r = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);
        coord_last_left_sample = l;
        coord_last_right_sample = r;
    }
}

/* **** Wireless Core **** */
static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_node_t *node;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;

static int32_t tx_timeslots[] = COORD_TIMESLOTS;
static int32_t rx_timeslots[] = NODE_TIMESLOTS;

/* There is a unidirectional link for audio and a bidirectional link for data. */
static swc_connection_t *tx_audio_conn;
static swc_connection_t *tx_data_conn;
static swc_connection_t *rx_data_conn;

/* **** Application Specific **** */
static facade_certification_mode_t certification_mode;
/* Variables supporting pairing between the two devices. */
static device_pairing_state_t device_pairing_state;
static pairing_cfg_t app_pairing_cfg;
static pairing_assigned_address_t pairing_assigned_address;
static pairing_discovery_list_t pairing_discovery_list[PAIRING_DISCOVERY_LIST_SIZE];

/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_init(void);
static void app_swc_core_init(pairing_assigned_address_t *app_pairing, swc_error_t *swc_err);
static void app_audio_core_init(void);

/* **** Callbacks **** */
static void conn_tx_audio_success_callback(void *conn);
static void conn_tx_data_success_callback(void *conn);
static void conn_rx_data_success_callback(void *conn);
static void i2s_rx_audio_complete_callback(void);
static void audio_process_callback(void);
static void data_callback(void);
static void pairing_process_callback(void);

/* **** Processing stages **** */
static void app_audio_core_fallback_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_packing_interface_init(sac_processing_interface_t *iface);
static void app_audio_core_packing_fallback_interface_init(sac_processing_interface_t *iface);

/* **** Button actions **** */
static void enter_pairing_mode(void);
static void unpair_device(void);
static void abort_pairing_procedure(void);

/* Fallback LED and terminal display. */
static void fallback_led_handler(void);
static bool should_print_stats(void);
static void print_stats(void);

static void wireless_send_data(void *transmitted_data, uint8_t size, swc_error_t *swc_err);
static uint16_t wireless_read_data(void *received_data, uint8_t size, swc_error_t *swc_err);

/* PUBLIC FUNCTIONS ***********************************************************/
int main(void)
{
    /* Initialize the board and all GPIOs and peripherals for minimal operations. */
    facade_board_init();

    /* Initialize wireless core context switch handler before pairing is available */
    facade_set_context_switch_handler(swc_connection_callbacks_processing_handler);

    /* Audio process timer initialization. */
    facade_audio_process_timer_init(audio_process_callback);

    /* Timer that updates statistics display every second and transmits button state to Node every 10 ms. */
    facade_data_timer_init(DATA_TX_PERIOD_MS);
    facade_data_timer_set_callback(data_callback);

    certification_mode = facade_get_coord_certification_mode();
    if (certification_mode != FACADE_CERTIF_NONE) {
        /* Init app in certification mode. */
        app_init();
        device_pairing_state = DEVICE_PAIRED;
        while (1) {
            /* Statistics are displayed at intervals set by the timer when paired; timer stops if unpaired. */
            if (should_print_stats()) {
                print_stats();
            }
        }
    }

    device_pairing_state = DEVICE_UNPAIRED;

    /* Pairing occurs automatically when the device boots. */
    enter_pairing_mode();


    while (1) {

        switch (device_pairing_state) {
        case DEVICE_UNPAIRED:
            /* When the device is unpaired, the only action possible for the user is to enter pairing mode. */
            facade_button_handling(enter_pairing_mode, NULL, NULL, NULL);
            break;
        case DEVICE_PAIRED:
            /* When the device is paired, normal operations are executed. */
            fallback_led_handler();
            facade_button_handling(unpair_device, NULL, NULL, NULL);
            break;
        default:
            facade_print_error_string("An unknown device pairing state occured.");
            while (1);
            break;
        }

        /* Statistics are displayed at intervals set by the timer when paired; timer stops if unpaired. */
        if (should_print_stats()) {
            print_stats();
        }
    }

    return 0;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[in]  app_pairing  Addresses received from the pairing process.
 *  @param[out] swc_err      Wireless Core error code.
 */
static void app_swc_core_init(pairing_assigned_address_t *app_pairing, swc_error_t *swc_err)
{
    uint8_t remote_address = pairing_discovery_list[PAIRING_DEVICE_ROLE_NODE].node_address;
    uint8_t local_address = pairing_discovery_list[PAIRING_DEVICE_ROLE_COORDINATOR].node_address;
    /* Threshold based on payload-only size; header is added by SAC */
    uint8_t fallback_thresholds[] = {FALLBACK_PAYLOAD_SIZE + sizeof(sac_header_t)
#if SAC_ENABLE_DEBUG_SEQ
                                     + 4
#endif
    };
    uint8_t fallback_cca_try_count[] = {SWC_CCA_AUDIO_FBK_TRY_COUNT};

    if (certification_mode != FACADE_CERTIF_NONE) {
        app_pairing->coordinator_address = 0x1;
        app_pairing->node_address = 0x2;
        app_pairing->pan_id = 0xABC;
        remote_address = 0x2;
        local_address = 0x1;
    }

    /* Initialize Wireless Core. */
    swc_cfg_t core_cfg = {
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = ARRAY_SIZE(channel_sequence),
        .concurrency_mode = SWC_CONCURRENCY_MODE_HIGH_PERFORMANCE,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE,
    };

    swc_init(core_cfg, facade_context_switch_trigger, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Initialize Node. */
    swc_node_cfg_t node_cfg = {
        .role = SWC_ROLE_COORDINATOR,
        .pan_id = app_pairing->pan_id,
        .coordinator_address = app_pairing->coordinator_address,
        .local_address = local_address,
    };

    node = swc_node_init(node_cfg, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Initialize radio. */
    swc_radio_module_init(node, SWC_RADIO_ID_1, true, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* **** TX Connections **** */
    /* ** TX Audio Connection ** */
    swc_connection_cfg_t tx_audio_conn_cfg = {
        .name = "TX Audio Connection",
        .source_address = local_address,
        .destination_address = remote_address,
        .max_payload_size = MAIN_CHANNEL_SWC_PAYLOAD_SIZE + sizeof(sac_header_t)
    #if SAC_ENABLE_DEBUG_SEQ
                    + 4
    #endif
        ,
        .queue_size = SWC_QUEUE_SIZE,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
    };

    /* ** TX Data Connection ** */
    swc_connection_cfg_t tx_data_conn_cfg = {
        .name = "TX Data Connection",
        .source_address = local_address,
        .destination_address = remote_address,
        .max_payload_size = MAX_DATA_PAYLOAD_SIZE,
        .queue_size = SWC_QUEUE_SIZE,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
    };

    if (certification_mode == FACADE_CERTIF_DATA) {
        /* Add data connection first to use it for certification mode. */
        tx_data_conn = swc_connection_init(node, tx_data_conn_cfg, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

        swc_connection_set_connection_priority(node, tx_data_conn, AUDIO_CONNECTION_PRIORITY, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

        tx_audio_conn = swc_connection_init(node, tx_audio_conn_cfg, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

        swc_connection_set_connection_priority(node, tx_audio_conn, DATA_CONNECTION_PRIORITY, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

    } else {
        if (certification_mode == FACADE_CERTIF_AUDIO_16_BIT) {
            /* Change the connection's max payload size when certifying compressed audio. */
            tx_audio_conn_cfg.max_payload_size = fallback_thresholds[0];
        }
        tx_audio_conn = swc_connection_init(node, tx_audio_conn_cfg, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

        swc_connection_set_connection_priority(node, tx_audio_conn, AUDIO_CONNECTION_PRIORITY, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

        tx_data_conn = swc_connection_init(node, tx_data_conn_cfg, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

        swc_connection_set_connection_priority(node, tx_data_conn, DATA_CONNECTION_PRIORITY, swc_err);
        ASSERT_SWC_STATUS(*swc_err);
    }
    /* Audio connection concurrency settings. */
    swc_connection_concurrency_cfg_t tx_audio_concurrency_cfg = {
        .enabled = true,
        .try_count = SWC_CCA_AUDIO_TRY_COUNT,
        .retry_time = SWC_CCA_RETRY_TIME,
        .fail_action = SWC_CCA_ABORT_TX,
    };

    swc_connection_set_concurrency_cfg(tx_audio_conn, &tx_audio_concurrency_cfg, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Audio connection fallback settings. */
    swc_connection_fallback_cfg_t fallback_cfg = {
        .enabled = true,
        .fallback_mode_count = 1,
        .thresholds = fallback_thresholds,
        .cca_try_count = fallback_cca_try_count,
    };

    swc_connection_set_fallback_cfg(tx_audio_conn, &fallback_cfg, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Audio connection RF channels settings. */
    swc_channel_cfg_t tx_audio_channel_cfg = {
        .tx_pulse_count = TX_AUDIO_PULSE_COUNT,
        .tx_pulse_width = TX_AUDIO_PULSE_WIDTH,
        .tx_pulse_gain = TX_AUDIO_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT,
    };

    swc_fallback_channel_cfg_t tx_audio_fallback_channel_cfg = {
        .tx_pulse_count = TX_AUDIO_FB_PULSE_COUNT,
        .tx_pulse_width = TX_AUDIO_FB_PULSE_WIDTH,
        .tx_pulse_gain = TX_AUDIO_FB_PULSE_GAIN,
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_audio_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_audio_conn, node, tx_audio_channel_cfg, swc_err);
        ASSERT_SWC_STATUS(*swc_err);

        swc_connection_add_fallback_channel(tx_audio_conn, node, tx_audio_channel_cfg, tx_audio_fallback_channel_cfg, i,
                                            0, swc_err);
        ASSERT_SWC_STATUS(*swc_err);
    }

    /* Audio connection callback settings. */
    swc_connection_set_tx_success_callback(tx_audio_conn, conn_tx_audio_success_callback, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Data connection concurrency settings. */
    swc_connection_concurrency_cfg_t tx_data_concurrency_cfg = {
        .enabled = true,
        .try_count = SWC_CCA_DATA_TRY_COUNT,
        .retry_time = SWC_CCA_RETRY_TIME,
        .fail_action = SWC_CCA_ABORT_TX,
    };

    swc_connection_set_concurrency_cfg(tx_data_conn, &tx_data_concurrency_cfg, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Data connection RF channels settings. */
    swc_channel_cfg_t tx_data_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT,
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_data_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_data_conn, node, tx_data_channel_cfg, swc_err);
        ASSERT_SWC_STATUS(*swc_err);
    }

    /* Data connection callback settings. */
    swc_connection_set_tx_success_callback(tx_data_conn, conn_tx_data_success_callback, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* ** RX Data Connection ** */
    swc_connection_cfg_t rx_data_conn_cfg = {
        .name = "RX Data Connection",
        .source_address = remote_address,
        .destination_address = local_address,
        .max_payload_size = MAX_DATA_PAYLOAD_SIZE,
        .queue_size = SWC_QUEUE_SIZE,
        .timeslot_id = rx_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_timeslots),
    };
    rx_data_conn = swc_connection_init(node, rx_data_conn_cfg, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Data connection concurrency settings. */
    swc_connection_concurrency_cfg_t rx_data_concurrency_cfg = {
        .enabled = true,
        .try_count = SWC_CCA_DATA_TRY_COUNT,
        .retry_time = SWC_CCA_RETRY_TIME,
        .fail_action = SWC_CCA_ABORT_TX,
    };

    swc_connection_set_concurrency_cfg(rx_data_conn, &rx_data_concurrency_cfg, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Data connection RF channels settings. */
    swc_channel_cfg_t rx_data_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT,
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_data_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_data_conn, node, rx_data_channel_cfg, swc_err);
        ASSERT_SWC_STATUS(*swc_err);
    }

    /* Data connection callback settings. */
    swc_connection_set_rx_success_callback(rx_data_conn, conn_rx_data_success_callback, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Handle certification mode. */
    swc_set_certification_mode(certification_mode != FACADE_CERTIF_NONE, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    /* Wireless Core setup. */
    swc_setup(node, swc_err);
    ASSERT_SWC_STATUS(*swc_err);
}

/** @brief Callback function when a previously sent audio frame has been ACK'd.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_tx_audio_success_callback(void *conn)
{
    (void)conn;

    facade_tx_audio_conn_status();

    /* Trigger audio process. */
    facade_audio_process_timer_trigger();
}

/** @brief Callback function when a previously sent data frame has been ACK'd.
 *
 *  @note This function is empty, but can be filled by users in any way they see fit.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_tx_data_success_callback(void *conn)
{
    (void)conn;
}

/** @brief Callback function when a data frame has been successfully received on data connection.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_data_success_callback(void *conn)
{
    (void)conn;

    sac_status_t sac_status = SAC_OK;
    swc_error_t swc_err = SWC_ERR_NONE;
    user_data_t received_user_data = {0};
    uint16_t read_data_size;

    /* Get received payload. */
    read_data_size = wireless_read_data(&received_user_data, sizeof(received_user_data), &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    if (read_data_size > 0) {
        /* Depending on the requested button state from the Node, the specified LED turns on or off. */
        if (received_user_data.button_state == false) {
            facade_empty_payload_received_status();
        } else {
            facade_payload_received_status();
        }

        /* The fallback state is updated. */
        sac_fallback_set_rx_link_margin(&sac_fallback_instance, received_user_data.link_margin, &sac_status);
        ASSERT_SAC_STATUS(sac_status);
    }
}

/** @brief Initialize the Audio Core.
 */
static void app_audio_core_init(void)
{
    sac_status_t sac_status = SAC_OK;

    sac_endpoint_interface_t i2s_producer_iface = {0};
    sac_endpoint_interface_t swc_consumer_iface = {0};

    sac_processing_interface_t fallback_iface = {0};
    sac_processing_interface_t packing_iface = {0};
    sac_processing_interface_t packing_fallback_iface = {0};

    sac_endpoint_swc_init(NULL, &swc_consumer_iface);
    sac_facade_codec_endpoint_init(&i2s_producer_iface, NULL);
    facade_set_sai_complete_callback(NULL, i2s_rx_audio_complete_callback);

    app_audio_core_fallback_interface_init(&fallback_iface);
    app_audio_core_packing_interface_init(&packing_iface);
    app_audio_core_packing_fallback_interface_init(&packing_fallback_iface);

    swc_consumer_instance.connection = tx_audio_conn;

    /* Initialize Audio Core. */
    /* Fill memory pool with pattern before init to catch overwrites */
    memset(audio_memory_pool, 0xCD, sizeof(audio_memory_pool));
    audio_pool_guard_head = 0xDEADBEEF;
    audio_pool_guard_tail = 0xA5A5A5A5;
    facade_print_string("DBG_COORD: sac_init begin\n");
    coord_check_audio_pool_guards("before_sac_init");
    sac_cfg_t core_cfg = {
        .memory_pool = audio_memory_pool,
        .memory_pool_size = SAC_MEM_POOL_SIZE,
    };
    sac_init(core_cfg, &sac_status);
    facade_print_string("DBG_COORD: sac_init done\n");
    coord_check_audio_pool_guards("after_sac_init");
    ASSERT_SAC_STATUS(sac_status);

    /*
     * Audio Pipeline
     * ==============
     *
     ***** NORMAL MODE *****
     * Input:      Stereo stream of 48kHz/24-bit depth samples, encoded on 32 bits.
     * Processing: Packing from 32 bits to 24 bits audio samples.
     * Output:     Stereo stream at 48 kHz/24 bits is sent over the air to the Node.
     *
     * +-----+    +--------------------+    +-----+
     * | I2S | -> | Packing to 24 bits | -> | SWC |
     * +-----+    +--------------------+    +-----+
     *
     ***** FALLBACK MODE *****
     * Input:      Stereo stream of 48kHz/24-bit depth samples, encoded on 32 bits.
     * Processing: Packing from 32 bits to 16 bits audio samples.
     * Output:     Stereo stream at 48 kHz/16 bits is sent over the air to the Node.
     *
     * +-----+    +--------------------+    +-----+
     * | I2S | -> | Packing to 16 bits | -> | SWC |
     * +-----+    +--------------------+    +-----+
     */

    /* Initialize codec producer endpoint. */
    sac_endpoint_cfg_t i2s_producer_cfg = {
        .use_encapsulation = false,
        .delayed_action = true,
        .channel_count = MAIN_CHANNEL_CHANNEL_COUNT,
        .audio_payload_size = MAIN_CHANNEL_I2S_PAYLOAD_SIZE,
        .queue_size = 4,
    };
    facade_print_string("DBG_COORD: endpoint i2s_producer init\n");
    i2s_producer = sac_endpoint_init(NULL, "I2S EP (Producer)", i2s_producer_iface, i2s_producer_cfg, &sac_status);
    coord_check_audio_pool_guards("after_i2s_producer");
    ASSERT_SAC_STATUS(sac_status);

    sac_fallback_instance = sac_fallback_get_defaults();
    sac_fallback_instance.connection = tx_audio_conn;
    sac_fallback_instance.is_tx_device = true;
    sac_fallback_instance.cca_max_try_count = SWC_CCA_AUDIO_FBK_TRY_COUNT;
    sac_fallback_instance.get_tick = facade_get_tick_ms;
    sac_fallback_instance.tick_frequency_hz = 1000;
    facade_print_string("DBG_COORD: processing fallback init\n");
    sac_fallback_processing = sac_processing_stage_init(&sac_fallback_instance, "Fallback TX", fallback_iface,
                                                        &sac_status);
    coord_check_audio_pool_guards("after_fallback_proc");
    ASSERT_SAC_STATUS(sac_status);

    /* Processing stage that packs into 24 bits before sending if fallback is deactivated. */
    audio_packing_instance.packing_mode = SAC_PACK_24BITS;
    facade_print_string("DBG_COORD: processing packing24 init\n");
    sac_packing_processing = sac_processing_stage_init((void *)&audio_packing_instance, "Audio Fallback Packing",
                                                       packing_iface, &sac_status);
    coord_check_audio_pool_guards("after_pack24_proc");
    ASSERT_SAC_STATUS(sac_status);

    /* Processing stage that packs into 16 bits before sending if fallback is activated. */
    audio_packing_fallback_instance.packing_mode = SAC_PACK_24BITS_16BITS;
    facade_print_string("DBG_COORD: processing packing16 init\n");
    sac_packing_fallback_processing = sac_processing_stage_init((void *)&audio_packing_fallback_instance,
                                                                "Audio Packing", packing_fallback_iface, &sac_status);
    coord_check_audio_pool_guards("after_pack16_proc");
    ASSERT_SAC_STATUS(sac_status);

    /* Initialize SWC consumer endpoint. */
    sac_endpoint_cfg_t swc_consumer_cfg = {
        .use_encapsulation = true,
        .delayed_action = false,
        .channel_count = MAIN_CHANNEL_CHANNEL_COUNT,
        .audio_payload_size = MAIN_CHANNEL_SWC_PAYLOAD_SIZE,
        .queue_size = MAIN_CHANNEL_LATENCY_QUEUE_SIZE,
    };
    facade_print_string("DBG_COORD: endpoint swc_consumer init\n");
    swc_consumer = sac_endpoint_init((void *)&swc_consumer_instance, "SWC EP (Consumer)", swc_consumer_iface,
                                     swc_consumer_cfg, &sac_status);
    coord_check_audio_pool_guards("after_swc_consumer");
    ASSERT_SAC_STATUS(sac_status);

    /* Initialize audio pipeline. */
    sac_pipeline_cfg_t pipeline_cfg = {
        .do_initial_buffering = true,
    };
    facade_print_string("DBG_COORD: pipeline init\n");
    sac_pipeline = sac_pipeline_init("I2S -> SWC", i2s_producer, pipeline_cfg, swc_consumer, &sac_status);
    coord_check_audio_pool_guards("after_pipeline_init");
    ASSERT_SAC_STATUS(sac_status);

    /* Add processing stages to the audio pipeline. */
    facade_print_string("DBG_COORD: add fallback proc\n");
    sac_pipeline_add_processing(sac_pipeline, sac_fallback_processing, &sac_status);
    coord_check_audio_pool_guards("after_add_fallback");
    ASSERT_SAC_STATUS(sac_status);

    facade_print_string("DBG_COORD: add pack24 proc\n");
    sac_pipeline_add_processing(sac_pipeline, sac_packing_processing, &sac_status);
    coord_check_audio_pool_guards("after_add_pack24");
    ASSERT_SAC_STATUS(sac_status);

    facade_print_string("DBG_COORD: add pack16 proc\n");
    sac_pipeline_add_processing(sac_pipeline, sac_packing_fallback_processing, &sac_status);
    coord_check_audio_pool_guards("after_add_pack16");
    ASSERT_SAC_STATUS(sac_status);

    /* Setup audio pipeline. */
    facade_print_string("DBG_COORD: pipeline setup\n");
    sac_pipeline_setup(sac_pipeline, &sac_status);
    coord_check_audio_pool_guards("after_pipeline_setup");
    ASSERT_SAC_STATUS(sac_status);
}

/** @brief Initialize the audio fallback processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_fallback_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_fallback_init;
    iface->ctrl = NULL;
    iface->process = sac_fallback_process;
    iface->gate = NULL;
}

/** @brief Initialize the audio packing processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_packing_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_packing_init;
    iface->ctrl = sac_packing_ctrl;
    iface->process = sac_packing_process;
    iface->gate = sac_fallback_gate_is_fallback_off;
}

/** @brief Initialize the audio fallback packing processing stage interface.
 *
 *  @param[out] iface  Processing interface.
 */
static void app_audio_core_packing_fallback_interface_init(sac_processing_interface_t *iface)
{
    iface->init = sac_packing_init;
    iface->ctrl = sac_packing_ctrl;
    iface->process = sac_packing_process;
    iface->gate = sac_fallback_gate_is_fallback_on;
}

/** @brief Update the fallback LED indicator.
 */
static void fallback_led_handler(void)
{
    sac_status_t sac_status = SAC_OK;

    facade_fallback_status(sac_fallback_is_active(&sac_fallback_instance, &sac_status));
    ASSERT_SAC_STATUS(sac_status);
}

/** @brief SAI DMA RX complete callback.
 *
 *  This receives audio packets from the codec. It needs to be executed every time a DMA transfer from the codec is
 *  completed in order to keep recording audio.
 */
static void i2s_rx_audio_complete_callback(void)
{
    sac_status_t sac_status = SAC_OK;

    /* The codec produces audio samples when it receives input audio. */
    sac_pipeline_produce(sac_pipeline, &sac_status);
    ASSERT_SAC_STATUS(sac_status);

    /* Trigger audio process. */
    facade_audio_process_timer_trigger();
}

/** @brief Callback handling the audio process that triggers with the app timer.
 */
static void audio_process_callback(void)
{
    sac_status_t sac_status = SAC_OK;
    uint32_t buffer_load = 0;

    buffer_load = sac_pipeline_get_producer_buffer_load(sac_pipeline, &sac_status);
    ASSERT_SAC_STATUS(sac_status);

    if (buffer_load > 0) {
        /* Processing stages of the pipeline are executed. */
        sac_pipeline_process(sac_pipeline, &sac_status);
        ASSERT_SAC_STATUS(sac_status);
    }

    buffer_load = sac_pipeline_get_consumer_buffer_load(sac_pipeline, &sac_status);
    ASSERT_SAC_STATUS(sac_status);

    if (buffer_load > 0) {
        /* The SWC consumes audio samples produced by the codec. */
        sac_pipeline_consume(sac_pipeline, &sac_status);
        ASSERT_SAC_STATUS(sac_status);
    }
}

/** @brief Check if stats should be printed.
 *
 *  @retval 0  Stats should not be printed.
 *  @retval 1  Stats should be printed.
 */
static bool should_print_stats(void)
{
    static uint32_t tick_start;
    uint32_t current_tick = facade_get_tick_ms();

    if (device_pairing_state != DEVICE_PAIRED) {
        tick_start = current_tick;
        return false;
    }

    if ((current_tick - tick_start) >= PRINT_INTERVAL_MS) {
        tick_start = current_tick;
        return true;
    }

    return false;
}

/** @brief Print the audio and wireless statistics.
 */
static void print_stats(void)
{
    if (device_pairing_state != DEVICE_PAIRED) {
        return;
    }

    static char stats_string[STATS_ARRAY_LENGTH];
    int string_length = 0;
    sac_status_t sac_status = SAC_OK;
    swc_error_t swc_err = SWC_ERR_NONE;

    const char *device_str = "\n<   COORDINATOR   >\n\r";
    const char *audio_stats_str = "\n<<  Audio Core Statistics  >>\n\r";
    const char *fallback_stats_str = "\n<<  Fallback Statistics  >>\n\r";
    const char *wireless_stats_str = "\n<<  Wireless Core Statistics  >>\n\r";
    const char *debug_stats_str = "\n<<  Debug (Seq)  >>\n\r";

    memset(stats_string, 0, sizeof(stats_string));

    /* ** Device Prelude ** */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, device_str);

    if (certification_mode != FACADE_CERTIF_NONE) {
        string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length,
                                  "Cert. Mode: %i\r\n", certification_mode);
    }

    /* ** Audio statistics ** */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, audio_stats_str);
    sac_pipeline_update_stats(sac_pipeline, &sac_status);
    ASSERT_SAC_STATUS(sac_status);
    string_length += sac_pipeline_format_stats(sac_pipeline, stats_string + string_length,
                                               sizeof(stats_string) - string_length, &sac_status);
    ASSERT_SAC_STATUS(sac_status);

    /* ** Audio fallback statistics ** */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, fallback_stats_str);
    string_length += sac_fallback_format_stats(&sac_fallback_instance, stats_string + string_length,
                                               sizeof(stats_string) - string_length, &sac_status);
    ASSERT_SAC_STATUS(sac_status);

    /* ** Wireless statistics ** */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, wireless_stats_str);
    swc_connection_update_stats(tx_audio_conn, &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    string_length += swc_connection_format_stats(tx_audio_conn, node, stats_string + string_length,
                                                 sizeof(stats_string) - string_length, &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    swc_connection_update_stats(tx_data_conn, &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    string_length += swc_connection_format_stats(tx_data_conn, node, stats_string + string_length,
                                                 sizeof(stats_string) - string_length, &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    swc_connection_update_stats(rx_data_conn, &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    string_length += swc_connection_format_stats(rx_data_conn, node, stats_string + string_length,
                                                 sizeof(stats_string) - string_length, &swc_err);
    ASSERT_SWC_STATUS(swc_err);


    /* ** Debug sequence statistics ** */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length, debug_stats_str);

    /* Get SWC statistics for TX audio connection (use update API, returns internal stats) */
    swc_statistics_t *pstats = swc_connection_update_stats(tx_audio_conn, &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    /* Calculate packet loss statistics */
    uint32_t tx_seq_total = coord_last_tx_seq;
    uint32_t swc_sent_acked = pstats ? pstats->packet_sent_and_acked_count : 0u;
    uint32_t swc_sent_not_acked = pstats ? pstats->packet_sent_and_not_acked_count : 0u;
    uint32_t swc_total_sent = swc_sent_acked + swc_sent_not_acked;
    uint32_t swc_dropped = pstats ? pstats->packet_dropped_count : 0u;
    uint32_t swc_cca_fail = pstats ? pstats->cca_fail_count : 0u;
    uint32_t swc_cca_try_fail = pstats ? pstats->cca_try_fail_count : 0u;

    double ack_rate = (swc_total_sent > 0u) ? (100.0 * (double)swc_sent_acked / (double)swc_total_sent) : 0.0;
    double loss_rate = (swc_total_sent > 0u) ? (100.0 * (double)swc_sent_not_acked / (double)swc_total_sent) : 0.0;
    /* Approx attempts per transmitted packet: 1 pass + average failed tries before pass */
    double attempts_per_pkt = (swc_total_sent > 0u) ? (1.0 + ((double)swc_cca_try_fail / (double)swc_total_sent)) : 0.0;

    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length,
                              "Seq Total: %lu (Next: %lu)\r\n",
                              (unsigned long)tx_seq_total,
                              (unsigned long)(tx_seq_total + 1));

    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length,
                              "SWC Sent: %lu | ACKed: %lu (%.1f%%) | Not ACKed: %lu (%.1f%%)\r\n",
                              (unsigned long)swc_total_sent,
                              (unsigned long)swc_sent_acked, ack_rate,
                              (unsigned long)swc_sent_not_acked, loss_rate);

    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length,
                              "SWC Dropped: %lu | CCA Fail: %lu\r\n",
                              (unsigned long)swc_dropped,
                              (unsigned long)swc_cca_fail);

    /* Attempts/Packet and link quality */
    double rssi_db = pstats ? ((double)pstats->rssi_avg / 10.0) : 0.0;
    double rnsi_db = pstats ? ((double)pstats->rnsi_avg / 10.0) : 0.0;
    double lm_db = pstats ? ((double)pstats->link_margin_avg / 10.0) : 0.0;
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length,
                              "Attempts/Packet: %.2f | RSSI: %.1f dB | RNSI: %.1f dB | LM: %.1f dB\r\n",
                              attempts_per_pkt, rssi_db, rnsi_db, lm_db);

    /* Print I2S sample debug info */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length,
                              "I2S OUT L: 0x%08lX R: 0x%08lX Size: %u\r\n",
                              (unsigned long)coord_last_left_sample,
                              (unsigned long)coord_last_right_sample,
                              (unsigned int)coord_last_payload_size);

    /* Placeholder for Node RX stats - will be populated when data link reports them */
    string_length += snprintf(stats_string + string_length, sizeof(stats_string) - string_length,
                              "Node: watch g_node_dbg (seq_last/seq_expected/seq_lost)\r\n");

    facade_print_string(stats_string);
}

/** @brief Callback sends the button state every 10 ms.
 */
static void data_callback(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    user_data_t transmitted_user_data = {0};

    /* Send the state of the button to the Node (The Link margin is not used). */
    transmitted_user_data.button_state = facade_read_button_state();
    wireless_send_data(&transmitted_user_data, sizeof(transmitted_user_data), &swc_err);
}

/** @brief Enter Pairing Mode using the Pairing Module.
 */
static void enter_pairing_mode(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    pairing_error_t pairing_err = PAIRING_ERR_NONE;
    pairing_event_t pairing_event = PAIRING_EVENT_NONE;

    facade_notify_enter_pairing();

    /* The wireless core must be stopped before starting the pairing procedure. */
    if (swc_get_status() == SWC_STATUS_RUNNING) {
        swc_disconnect(&swc_err);
        ASSERT_SWC_STATUS(swc_err);
    }

    /* Give the information to the Pairing Module. */
    app_pairing_cfg.app_code = PAIRING_APP_CODE;
    app_pairing_cfg.timeout_sec = PAIRING_TIMEOUT_IN_SECONDS;
    app_pairing_cfg.application_callback = pairing_process_callback;
    app_pairing_cfg.memory_pool = swc_memory_pool;
    app_pairing_cfg.memory_pool_size = SWC_MEM_POOL_SIZE;
    app_pairing_cfg.context_switch_callback = facade_context_switch_trigger;
    pairing_event = pairing_coordinator_start(&app_pairing_cfg, &pairing_assigned_address, pairing_discovery_list,
                                              PAIRING_DISCOVERY_LIST_SIZE, &pairing_err);
    if (pairing_err != PAIRING_ERR_NONE) {
        facade_print_error_string("An error occured during the pairing process.");
        while (1);
    }

    /* Handle the pairing events. */
    switch (pairing_event) {
    case PAIRING_EVENT_SUCCESS:
        /* Indicate that the pairing process was successful. */
        facade_notify_pairing_successful();

        app_init();
        device_pairing_state = DEVICE_PAIRED;

        break;
    case PAIRING_EVENT_TIMEOUT:
    case PAIRING_EVENT_INVALID_APP_CODE:
    case PAIRING_EVENT_ABORT:
    default:
        /* Indicate that the pairing process was unsuccessful. */
        facade_notify_not_paired();
        device_pairing_state = DEVICE_UNPAIRED;
        break;
    }
}

/** @brief Unpair the device. This will reset its discovery list.
 */
static void unpair_device(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    sac_status_t sac_status = SAC_OK;

    device_pairing_state = DEVICE_UNPAIRED;

    /* Stop timers. */
    facade_audio_process_timer_stop();
    facade_data_timer_stop();

    /* Disconnect the Wireless Core. */
    swc_disconnect(&swc_err);
    ASSERT_SWC_STATUS(swc_err);

    tx_audio_conn = NULL;
    tx_data_conn = NULL;
    rx_data_conn = NULL;

    /* Reset the pairing discovery list. */
    memset(pairing_discovery_list, 0, sizeof(pairing_discovery_list));

    /* Stop the audio pipeline. */
    sac_pipeline_stop(sac_pipeline, &sac_status);
    ASSERT_SAC_STATUS(sac_status);
    sac_pipeline = NULL;

    facade_audio_deinit();

    /* Indicate that the device is unpaired. */
    facade_led_all_off();
    facade_notify_not_paired();
}

/** @brief Pairing process callback called during pairing.
 */
static void pairing_process_callback(void)
{
    /*
     * Note: The button press will only be detected when the pairing executes the registered pairing process callback,
     *       which might take a variable amount of time.
     */
    facade_button_handling(abort_pairing_procedure, NULL, NULL, NULL);
}

/** @brief Abort the pairing procedure.
 */
static void abort_pairing_procedure(void)
{
    pairing_abort();
}

/** @brief Send data with a specific connection.
 *
 *  @param[in]  transmitted_data  Data to be sent over the air.
 *  @param[in]  size              Size of the data to be sent over the air.
 *  @param[out] swc_err           Wireless Core error code.
 */
static void wireless_send_data(void *transmitted_data, uint8_t size, swc_error_t *swc_err)
{
    uint8_t *buffer = NULL;

    /* Get buffer from queue to hold data. */
    swc_connection_get_payload_buffer(tx_data_conn, &buffer, swc_err);
    if ((*swc_err != SWC_ERR_NONE) || (buffer == NULL)) {
        return;
    }

    /* Format the new payload. */
    if (transmitted_data != NULL) {
        memcpy(buffer, transmitted_data, size);
    }

    /* Send the payload through the Wireless Core. */
    swc_connection_send(tx_data_conn, buffer, size, swc_err);
    ASSERT_SWC_STATUS(*swc_err);
}

/** @brief Read data from a specific connection.
 *
 *  @param[out] received_data  Pointer to data buffer to write to.
 *  @param[in]  size           Size of the data buffer.
 *  @param[out] swc_err        Wireless Core error code.
 *
 *  @return Size of the data read.
 */
static uint16_t wireless_read_data(void *received_data, uint8_t size, swc_error_t *swc_err)
{
    uint8_t *payload = NULL;
    uint16_t payload_size = 0;

    /* Read received data. */
    payload_size = swc_connection_receive(rx_data_conn, &payload, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    if (payload_size > size) {
        return 0;
    }

    if (received_data != NULL) {
        memcpy(received_data, payload, payload_size);
    }

    /* Free the payload memory. */
    swc_connection_receive_complete(rx_data_conn, swc_err);
    ASSERT_SWC_STATUS(*swc_err);

    return payload_size;
}

/** @brief Initialize the application.
 */
static void app_init(void)
{
    swc_error_t swc_err = SWC_ERR_NONE;
    sac_status_t sac_status = SAC_OK;

    /* Initialize Wireless Core. */
    app_swc_core_init(&pairing_assigned_address, &swc_err);
    ASSERT_SWC_STATUS(swc_err);

    /* Connect the Wireless Core. */
    swc_connect(&swc_err);
    ASSERT_SWC_STATUS(swc_err);

    /* Initialize Audio Core. */
    app_audio_core_init();

    /* Initialize GPIOs and peripherals for audio operations. */
    facade_audio_coord_init();

    /* Start the audio pipeline. */
    sac_pipeline_start(sac_pipeline, &sac_status);
    ASSERT_SAC_STATUS(sac_status);

    /* Start timer used for audio process. */
    facade_audio_process_timer_start();

    /* Start data and statistics timer. */
    facade_data_timer_start();
}

void sac_error_handler(sac_status_t sac_status)
{
    char buffer[ERROR_MESSAGE_BUFFER_SIZE];

    sprintf(buffer, "SAC Error! Code: %d\n\r", sac_status);
    facade_print_error_string(buffer);

    while (1);
}

void swc_error_handler(swc_error_t swc_status)
{
    char buffer[ERROR_MESSAGE_BUFFER_SIZE];

    sprintf(buffer, "SWC Error ! Code: %d\n\r", swc_status);
    facade_print_error_string(buffer);

    while (1);
}
