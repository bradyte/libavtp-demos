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
 * Based on validation logic from crf-listener.c (Intel Corporation).
 * Implements IEEE 1722-2016 CRF packet validation.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "avtp.h"
#include "crf-common.h"
#include "phc-utils.h"

void crf_config_init(struct crf_stream_config *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->stream_id = CRF_STREAM_ID_DEFAULT;
	cfg->sample_rate = CRF_SAMPLE_RATE_48K;
	cfg->timestamps_per_pkt = CRF_TIMESTAMPS_PER_PKT;
	cfg->verbose = false;
}

bool crf_pdu_validate(struct avtp_crf_pdu *pdu,
		      const struct crf_stream_config *cfg,
		      uint8_t *seq_num)
{
	int res;
	uint32_t val32;
	uint64_t val64;
	struct avtp_common_pdu *common = (struct avtp_common_pdu *)pdu;

	if (!pdu || !cfg)
		return false;

	res = avtp_pdu_get(common, AVTP_FIELD_SUBTYPE, &val32);
	if (res < 0 || val32 != AVTP_SUBTYPE_CRF) {
		if (cfg->verbose)
			fprintf(stderr, "CRF: Invalid subtype\n");
		return false;
	}

	res = avtp_pdu_get(common, AVTP_FIELD_VERSION, &val32);
	if (res < 0 || val32 != 0) {
		if (cfg->verbose)
			fprintf(stderr, "CRF: Invalid version\n");
		return false;
	}

	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_SV, &val64);
	if (res < 0 || val64 != 1) {
		if (cfg->verbose)
			fprintf(stderr, "CRF: sv not set\n");
		return false;
	}

	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_STREAM_ID, &val64);
	if (res < 0 || val64 != cfg->stream_id) {
		if (cfg->verbose)
			fprintf(stderr, "CRF: Stream ID mismatch\n");
		return false;
	}

	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_TYPE, &val64);
	if (res < 0 || val64 != AVTP_CRF_TYPE_AUDIO_SAMPLE) {
		if (cfg->verbose)
			fprintf(stderr, "CRF: Invalid type\n");
		return false;
	}

	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_BASE_FREQ, &val64);
	if (res < 0 || val64 != cfg->sample_rate) {
		if (cfg->verbose)
			fprintf(stderr, "CRF: Sample rate mismatch\n");
		return false;
	}

	if (seq_num) {
		res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_SEQ_NUM, &val64);
		if (res < 0)
			return false;

		if (val64 != *seq_num && cfg->verbose)
			fprintf(stderr, "CRF: Sequence mismatch: expected %u, got %" PRIu64 "\n",
				*seq_num, val64);

		*seq_num = val64 + 1;
	}

	return true;
}

int crf_pdu_get_timestamps(struct avtp_crf_pdu *pdu,
			   uint64_t *timestamps,
			   size_t max_timestamps)
{
	int res;
	uint64_t data_len;
	size_t count, i;

	if (!pdu || !timestamps || max_timestamps == 0)
		return -1;

	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_CRF_DATA_LEN, &data_len);
	if (res < 0)
		return -1;

	count = data_len / sizeof(uint64_t);
	if (count > max_timestamps)
		count = max_timestamps;

	for (i = 0; i < count; i++)
		timestamps[i] = be64toh(pdu->crf_data[i]);

	return count;
}

uint64_t crf_get_phc_time_utc(clockid_t phc_clk, int tai_offset_sec)
{
	uint64_t phc_tai_ns = phc_gettime_ns(phc_clk);

	/* PHC is in TAI (International Atomic Time).
	 * Convert to UTC by subtracting TAI-UTC offset.
	 * FIXME: Remove when talker uses PHC directly. */
	return phc_tai_ns - ((uint64_t)tai_offset_sec * NSEC_PER_SEC);
}

uint64_t crf_get_realtime_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);
	return (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}
