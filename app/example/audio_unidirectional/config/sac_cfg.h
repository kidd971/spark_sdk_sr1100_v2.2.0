/** @file  sac_cfg.h
 *  @brief Configuration constants for the SPARK Audio Core.
 *
 *  @copyright Copyright (C) 2025 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_CFG_H_
#define SAC_CFG_H_

#include "sac_compression.h"
#include "sac_utils.h"

/* CONSTANTS ******************************************************************/
/* **** I2S Settings. **** */

/* The I2S sample rate will be shared by all audio streams.
 * SRC Audio processing is required if a stream uses a different sample rate.
 */
#define I2S_SAMPLE_RATE_HZ 48000
/* I2S bit depth is defined by the capabilities of the DMA transfer.
 * The DMA can transport data on either a byte (8-bit), a half-word (16-bit) or a word (32-bit)
 */
#define I2S_BIT_DEPTH 32
/* Latency induced by the codec's ADC and DAC. */
#define CODEC_LATENCY_MS 1

/* **** Main Channel settings. **** */

#define MAIN_CHANNEL_SAMPLE_RATE_HZ I2S_SAMPLE_RATE_HZ
#define MAIN_CHANNEL_SAMPLE_COUNT   13
#define MAIN_CHANNEL_CHANNEL_COUNT  2
#define MAIN_CHANNEL_BIT_DEPTH      24
#define MAIN_CHANNEL_LATENCY_MS     5

/* Calculated values. */
#define MAIN_CHANNEL_SWC_PAYLOAD_SIZE \
    SAC_CALCULATE_PAYLOAD_SIZE(MAIN_CHANNEL_SAMPLE_COUNT, MAIN_CHANNEL_CHANNEL_COUNT, MAIN_CHANNEL_BIT_DEPTH)
#define MAIN_CHANNEL_I2S_PAYLOAD_SIZE \
    SAC_CALCULATE_PAYLOAD_SIZE(MAIN_CHANNEL_SAMPLE_COUNT, MAIN_CHANNEL_CHANNEL_COUNT, I2S_BIT_DEPTH)
/* Size of the latency queue used by the Audio Core for the main channel. */
#define MAIN_CHANNEL_LATENCY_QUEUE_SIZE                                                                    \
    SAC_CALCULATE_LATENCY_QUEUE_SIZE(MAIN_CHANNEL_LATENCY_MS, CODEC_LATENCY_MS, MAIN_CHANNEL_SAMPLE_COUNT, \
                                     MAIN_CHANNEL_SAMPLE_RATE_HZ)

#endif /* SAC_CFG_H_ */
