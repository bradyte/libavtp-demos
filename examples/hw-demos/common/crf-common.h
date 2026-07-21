/*
 * Copyright (c) 2024, Tom
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Intel Corporation nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* CRF Common Utilities
 *
 * Shared CRF packet validation and extraction functions based on IEEE 1722.
 * Extracted from crf-listener.c to avoid code duplication across tools.
 */

#pragma once

#include <endian.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "avtp_crf.h"

/* Standard CRF stream parameters per IEEE 1722-2016 Table 28 */
#define CRF_STREAM_ID_DEFAULT   0xAABBCCDDEEFF0002
#define CRF_SAMPLE_RATE_48K     48000
#define CRF_TIMESTAMPS_PER_PKT  6
#define CRF_PACKET_RATE_HZ      50
#define CRF_DATA_LEN            (sizeof(uint64_t) * CRF_TIMESTAMPS_PER_PKT)
#define CRF_PDU_SIZE            (sizeof(struct avtp_crf_pdu) + CRF_DATA_LEN)

/* Time conversion constant */
#define NSEC_PER_SEC            1000000000ULL

/* CRF stream configuration */
struct crf_stream_config {
	uint64_t stream_id;
	uint32_t sample_rate;
	uint8_t timestamps_per_pkt;
	bool verbose;  /* Print validation errors to stderr */
};

/* Initialize configuration with defaults (48kHz audio sample clock) */
void crf_config_init(struct crf_stream_config *cfg);

/* Validate CRF PDU against expected configuration
 * Updates seq_num on success (can be NULL if not tracking sequence) */
bool crf_pdu_validate(struct avtp_crf_pdu *pdu,
		      const struct crf_stream_config *cfg,
		      uint8_t *seq_num);

/* Extract all timestamps from CRF PDU
 * Returns number of timestamps extracted, or -1 on error */
int crf_pdu_get_timestamps(struct avtp_crf_pdu *pdu,
			   uint64_t *timestamps,
			   size_t max_timestamps);

/* Get first timestamp (ts0) - the presentation time reference */
static inline uint64_t crf_pdu_get_ts0(struct avtp_crf_pdu *pdu)
{
	return be64toh(pdu->crf_data[0]);
}

/* Get PHC time in UTC (compensates for TAI offset)
 * Requires PHC to be opened via phc_open() first */
uint64_t crf_get_phc_time_utc(clockid_t phc_clk, int tai_offset_sec);

/* Get current time from CLOCK_REALTIME (assumed PTP-synced via phc2sys).
 * Returns nanoseconds since Unix epoch. */
uint64_t crf_get_realtime_ns(void);
