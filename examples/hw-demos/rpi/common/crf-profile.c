/*
 * Copyright (c) 2018, Intel Corporation
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

/* CRF Stream Profile
 *
 * Field validation ported from the reference crf-listener.c (Intel), extended
 * with the two checks it omits and this application depends on:
 *
 *   pull                - the sample rate multiplier. A stream carrying
 *                         base_freq 48000 with pull MULT_BY_1_OVER_1_001 is a
 *                         47952.05 Hz clock. Accepting it on a base_freq match
 *                         alone hands the servo a 1000 ppm error.
 *   timestamp_interval  - together with base_freq this fixes the feedback edge
 *                         rate the hardware generates. A stream that changes it
 *                         silently breaks the 1:1 CRF-to-edge invariant the
 *                         whole signal path is built on.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "avtp.h"
#include "crf-profile.h"

void crf_profile_init(struct crf_profile *profile)
{
	if (!profile)
		return;

	memset(profile, 0, sizeof(*profile));

	/* IEEE 1722-2016 Table 28 recommendation for a 48 kHz audio clock. */
	profile->stream_id	    = 0xAABBCCDDEEFF0002;
	profile->base_freq	    = 48000;
	profile->timestamp_interval = 160;
	profile->type		    = AVTP_CRF_TYPE_AUDIO_SAMPLE;
	profile->pull		    = AVTP_CRF_PULL_MULT_BY_1;
	profile->timestamps_per_pdu = CRF_MAX_TIMESTAMPS_PER_PDU;
}

double crf_pull_ratio(uint8_t pull)
{
	switch (pull) {
	case AVTP_CRF_PULL_MULT_BY_1:
		return 1.0;
	case AVTP_CRF_PULL_MULT_BY_1_OVER_1_001:
		return 1.0 / 1.001;
	case AVTP_CRF_PULL_MULT_BY_1_001:
		return 1.001;
	case AVTP_CRF_PULL_MULT_BY_24_OVER_25:
		return 24.0 / 25.0;
	case AVTP_CRF_PULL_MULT_BY_25_OVER_24:
		return 25.0 / 24.0;
	case AVTP_CRF_PULL_MULT_BY_1_OVER_8:
		return 1.0 / 8.0;
	default:
		return 0.0;
	}
}

void crf_profile_describe(const struct crf_profile *profile, char *buf,
			  size_t len)
{
	if (!profile || !buf || len == 0)
		return;

	snprintf(buf, len,
		 "stream 0x%016" PRIx64 "  %.3f Hz media clock  "
		 "interval %u  %.3f Hz edges  tol +/-%" PRId64 " us",
		 profile->stream_id,
		 crf_profile_media_hz(profile),
		 profile->timestamp_interval,
		 crf_profile_edge_hz(profile),
		 crf_profile_edge_tolerance_ns(profile) / 1000);
}

const char *crf_pdu_status_str(enum crf_pdu_status status)
{
	switch (status) {
	case CRF_PDU_OK:
		return "ok";
	case CRF_PDU_NOT_CRF:
		return "not a CRF PDU";
	case CRF_PDU_OTHER_STREAM:
		return "different stream";
	case CRF_PDU_MALFORMED:
		return "malformed";
	case CRF_PDU_PROFILE:
		return "profile mismatch";
	default:
		return "unknown";
	}
}

/* Record a field disagreement. Under CRF_STRICT every one rejects. Under
 * CRF_LENIENT only the checks the original validator performed reject; the
 * rest are recorded and the PDU is accepted, so behaviour matches the
 * pre-profile code exactly. */
static enum crf_pdu_status note_mismatch(struct crf_pdu_info *out,
					 const char *name, uint64_t got,
					 uint64_t want, bool legacy,
					 enum crf_strictness mode)
{
	out->bad_field = name;
	out->bad_got = got;
	out->bad_want = want;

	if (legacy || mode == CRF_STRICT)
		return CRF_PDU_PROFILE;

	return CRF_PDU_OK;
}

enum crf_pdu_status crf_pdu_parse(const struct avtp_crf_pdu *pdu, size_t len,
				  const struct crf_profile *profile,
				  enum crf_strictness mode,
				  struct crf_pdu_info *out)
{
	const struct avtp_common_pdu *common = (const struct avtp_common_pdu *)pdu;
	enum crf_pdu_status status;
	uint64_t val64;
	uint32_t val32;
	size_t i;
	int res;

	if (!pdu || !profile || !out)
		return CRF_PDU_MALFORMED;

	memset(out, 0, sizeof(*out));

	if (profile->timestamps_per_pdu == 0 ||
	    profile->timestamps_per_pdu > CRF_MAX_TIMESTAMPS_PER_PDU)
		return CRF_PDU_MALFORMED;

	if (len != crf_profile_pdu_size(profile))
		return CRF_PDU_MALFORMED;

	/* Subtype first: any other subtype is simply not our traffic, and on a
	 * shared segment that is the common case. Never worth reporting. */
	res = avtp_pdu_get(common, AVTP_FIELD_SUBTYPE, &val32);
	if (res < 0)
		return CRF_PDU_MALFORMED;
	if (val32 != AVTP_SUBTYPE_CRF)
		return CRF_PDU_NOT_CRF;

	/* Stream ID next, for the same reason: another talker's CRF stream is
	 * not an error condition. */
	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_STREAM_ID, &val64);
	if (res < 0)
		return CRF_PDU_MALFORMED;
	if (val64 != profile->stream_id)
		return CRF_PDU_OTHER_STREAM;

	/* Past this point the PDU claims to be our stream, so any disagreement
	 * is a real misconfiguration and worth surfacing. */
	res = avtp_pdu_get(common, AVTP_FIELD_VERSION, &val32);
	if (res < 0)
		return CRF_PDU_MALFORMED;
	if (val32 != 0)
		return note_mismatch(out, "version", val32, 0, true, mode);

	/* legacy marks the checks the pre-profile validator already performed.
	 * Those reject in both modes; the rest only under CRF_STRICT. */
	const struct {
		enum avtp_crf_field field;
		uint64_t expect;
		const char *name;
		bool legacy;
	} checks[] = {
		{ AVTP_CRF_FIELD_SV, 1, "sv", true },
		{ AVTP_CRF_FIELD_TYPE, profile->type, "type", true },
		{ AVTP_CRF_FIELD_BASE_FREQ, profile->base_freq, "base_freq",
		  true },
		{ AVTP_CRF_FIELD_FS, 0, "fs", false },
		{ AVTP_CRF_FIELD_PULL, profile->pull, "pull", false },
		{ AVTP_CRF_FIELD_TIMESTAMP_INTERVAL,
		  profile->timestamp_interval, "timestamp_interval", false },
	};

	for (i = 0; i < sizeof(checks) / sizeof(checks[0]); i++) {
		res = avtp_crf_pdu_get(pdu, checks[i].field, &val64);
		if (res < 0)
			return CRF_PDU_MALFORMED;
		if (val64 != checks[i].expect) {
			status = note_mismatch(out, checks[i].name, val64,
					       checks[i].expect,
					       checks[i].legacy, mode);
			if (status != CRF_PDU_OK)
				return status;
		}
	}

	/* crf_data_len is the one field whose legacy handling was not a plain
	 * equality test: the old extract path divided it by 8 and clamped the
	 * result to the buffer, so a PDU was usable whenever the field was big
	 * enough to fill it, and an over-long value was silently truncated.
	 * Under CRF_LENIENT that means too-small rejects and too-large is only
	 * reported. CRF_STRICT requires exactly the profile length. */
	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_CRF_DATA_LEN, &val64);
	if (res < 0)
		return CRF_PDU_MALFORMED;
	if (val64 != crf_profile_data_len(profile)) {
		bool undersized = val64 < crf_profile_data_len(profile);

		status = note_mismatch(out, "crf_data_len", val64,
				       crf_profile_data_len(profile),
				       undersized, mode);
		if (status != CRF_PDU_OK)
			return status;
	}

	/* Signalling fields: reported to the caller, never a rejection. */
	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_MR, &val64);
	if (res < 0)
		return CRF_PDU_MALFORMED;
	out->mr = val64 != 0;

	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_TU, &val64);
	if (res < 0)
		return CRF_PDU_MALFORMED;
	out->tu = val64 != 0;

	res = avtp_crf_pdu_get(pdu, AVTP_CRF_FIELD_SEQ_NUM, &val64);
	if (res < 0)
		return CRF_PDU_MALFORMED;
	out->seq_num = (uint8_t)val64;

	for (i = 0; i < profile->timestamps_per_pdu; i++)
		out->timestamps[i] = be64toh(pdu->crf_data[i]);
	out->count = profile->timestamps_per_pdu;

	return CRF_PDU_OK;
}

int crf_pdu_init(struct avtp_crf_pdu *pdu, const struct crf_profile *profile)
{
	size_t i;
	int res;

	if (!pdu || !profile)
		return -1;

	res = avtp_crf_pdu_init(pdu);
	if (res < 0)
		return -1;

	const struct {
		enum avtp_crf_field field;
		uint64_t value;
	} fields[] = {
		{ AVTP_CRF_FIELD_FS, 0 },
		{ AVTP_CRF_FIELD_TYPE, profile->type },
		{ AVTP_CRF_FIELD_STREAM_ID, profile->stream_id },
		{ AVTP_CRF_FIELD_PULL, profile->pull },
		{ AVTP_CRF_FIELD_BASE_FREQ, profile->base_freq },
		{ AVTP_CRF_FIELD_TIMESTAMP_INTERVAL,
		  profile->timestamp_interval },
		{ AVTP_CRF_FIELD_CRF_DATA_LEN, crf_profile_data_len(profile) },
	};

	for (i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
		res = avtp_crf_pdu_set(pdu, fields[i].field, fields[i].value);
		if (res < 0)
			return -1;
	}

	return 0;
}
