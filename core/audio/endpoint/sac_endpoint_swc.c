/** @file  sac_endpoint_swc.c
 *  @brief Implement Wireless Core audio endpoint initialization functions.
 *
 *  @copyright Copyright (C) 2024 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_endpoint_swc.h"

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static uint16_t ep_swc_action_produce(void *instance, uint8_t *samples, uint16_t size);
static uint16_t ep_swc_action_consume(void *instance, uint8_t *samples, uint16_t size);
static void ep_swc_consumer_start(void *instance);
static void ep_swc_producer_start(void *instance);
static void ep_swc_stop(void *instance);

/* Debug sequence instrumentation switch (default OFF for stock behavior) */
#ifndef SAC_ENABLE_DEBUG_SEQ
#define SAC_ENABLE_DEBUG_SEQ 0
#endif

/* Optional weak debug hooks implemented in application if needed */
__attribute__((weak)) void sac_debug_update_rx_seq(uint32_t seq) { (void)seq; }
__attribute__((weak)) void sac_debug_update_tx_seq(uint32_t seq) { (void)seq; }
/* Optional weak hook to observe SWC TX packet (header+payload before OTA) */
__attribute__((weak)) void sac_debug_update_swc_tx_packet(const uint8_t *packet, uint16_t size)
{
    (void)packet;
    (void)size;
}

#if SAC_ENABLE_DEBUG_SEQ
/* TX sequence counter */
static uint32_t g_swc_tx_seq = 0;
#endif

/* PUBLIC FUNCTIONS ***********************************************************/
void sac_endpoint_swc_init(sac_endpoint_interface_t *swc_producer_iface, sac_endpoint_interface_t *swc_consumer_iface)
{
    if (swc_producer_iface != NULL) {
        swc_producer_iface->action = ep_swc_action_produce;
        swc_producer_iface->start = ep_swc_producer_start;
        swc_producer_iface->stop = ep_swc_stop;
    }

    if (swc_consumer_iface != NULL) {
        swc_consumer_iface->action = ep_swc_action_consume;
        swc_consumer_iface->start = ep_swc_consumer_start;
        swc_consumer_iface->stop = ep_swc_stop;
    }
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Produce Endpoint of the SPARK Wireless Core.
 *
 *  @param[in]  instance  Endpoint instance.
 *  @param[out] samples   Produced samples.
 *  @param[in]  size      Size of samples to produce in bytes.
 *  @return Number of bytes produced.
 */
static uint16_t ep_swc_action_produce(void *instance, uint8_t *samples, uint16_t size)
{
    uint8_t *payload = NULL;
    uint16_t payload_size = 0; /* May include debug seq bytes when enabled */
    swc_error_t err = SWC_ERR_NONE;
    ep_swc_instance_t *inst = (ep_swc_instance_t *)instance;
    (void)size;

    payload_size = swc_connection_receive(inst->connection, &payload, &err);
    if (payload != NULL && payload_size) {
#if SAC_ENABLE_DEBUG_SEQ
        /* Expect format: [header][audio payload][optional 4-byte seq] */
        if (payload_size >= sizeof(sac_header_t)) {
            sac_header_t *hdr = (sac_header_t *)payload;
            uint16_t expected_total = sizeof(sac_header_t) + hdr->payload_size;
            if (payload_size >= expected_total + 4) {
                /* Extract sequence number */
                uint32_t seq;
                memcpy(&seq, payload + expected_total, 4);
                sac_debug_update_rx_seq(seq);
                /* Copy only header + audio payload to samples */
                memcpy(samples, payload, expected_total);
                swc_connection_receive_complete(inst->connection, &err);
                return expected_total; /* Strip debug bytes */
            }
        }
#endif
        /* Copy everything */
        memcpy(samples, payload, payload_size);
    }
    swc_connection_receive_complete(inst->connection, &err);
    return payload_size;
}

/** @brief Consume Endpoint of the SPARK Wireless Core.
 *
 *  @param[in] instance  Endpoint instance.
 *  @param[in] samples   Samples to consume.
 *  @param[in] size      Size of samples to consume in bytes.
 *  @return Number of bytes consumed.
 */
static uint16_t ep_swc_action_consume(void *instance, uint8_t *samples, uint16_t size)
{
    uint8_t *buf = NULL;
    swc_error_t err = SWC_ERR_NONE;
    ep_swc_instance_t *inst = (ep_swc_instance_t *)instance;

    /** When the fallback is activated, and the payload size is smaller, the variable allocation of memory causes
     *  fragmentation in the queue of the wireless.
     */
    swc_connection_get_payload_buffer(inst->connection, &buf, &err);
    if (buf == NULL) {
        return 0;
    }
    memcpy(buf, samples, size);
    /* Debug: inform app of the outgoing SWC packet content (header + payload) */
    sac_debug_update_swc_tx_packet(samples, size);
#if SAC_ENABLE_DEBUG_SEQ
    /* Append 4-byte sequence number after header+payload when size > 0 */
    if (size > 0) {
        uint32_t seq = g_swc_tx_seq++;
        memcpy(buf + size, &seq, 4);
        sac_debug_update_tx_seq(seq);
        swc_connection_send(inst->connection, buf, size + 4, &err);
        return size + 4;
    }
#endif
    swc_connection_send(inst->connection, buf, size, &err);
    return size;
}

/** @brief Start the consumer endpoint.
 *
 *  @param[in] instance  Endpoint instance.
 */
static void ep_swc_consumer_start(void *instance)
{
    uint8_t *buf = NULL;
    swc_error_t err = SWC_ERR_NONE;
    ep_swc_instance_t *inst = (ep_swc_instance_t *)instance;

    /* Fill wireless core queue with empty payload to maintain buffering latency. */
    swc_connection_get_payload_buffer(inst->connection, &buf, &err);
    while (buf != NULL) {
        swc_connection_send(inst->connection, buf, 0, &err);
        ASSERT_SWC_STATUS(err);

        swc_connection_get_payload_buffer(inst->connection, &buf, &err);
        if (err == SWC_ERR_NO_BUFFER_AVAILABLE) {
            /* Ignore SWC_ERR_NO_BUFFER_AVAILABLE error since it is expected. */
            continue;
        }
        ASSERT_SWC_STATUS(err);
    }
}

/** @brief Start the producer endpoint.
 *
 *  @param[in] instance  Endpoint instance.
 */
static void ep_swc_producer_start(void *instance)
{
    (void)instance;
}

/** @brief Stop the endpoint.
 *
 *  @param[in] instance  Endpoint instance.
 */
static void ep_swc_stop(void *instance)
{
    (void)instance;
}
