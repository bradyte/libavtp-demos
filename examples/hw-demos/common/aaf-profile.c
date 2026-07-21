/*
 * Copyright (c) 2019, Intel Corporation
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

/* AAF Stream Profile
 *
 * Field validation follows the reference aaf-listener.c, extended with the
 * two fields the talker sets but no listener checked:
 *
 *   bit_depth  a 24-bit stream carried in 32-bit containers has the same
 *              format code and the same data length as a 32-bit stream.
 *              Only bit_depth distinguishes them, and misreading one as the
 *              other scales every sample.
 *   sp         sparse mode changes which PDUs carry a valid timestamp. The
 *              presentation-time gate assumes every PDU does.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "avtp.h"
#include "aaf-profile.h"

void aaf_profile_init(struct aaf_profile *profile)
{
	if (!profile)
		return;

	memset(profile, 0, sizeof(*profile));

	profile->stream_id	= 0xAABBCCDDEEFF0002;
	profile->nsr		= AVTP_AAF_PCM_NSR_48KHZ;
	profile->format		= AVTP_AAF_FORMAT_INT_32BIT;
	profile->bit_depth	= 32;
	profile->sp		= AVTP_AAF_PCM_SP_NORMAL;
	profile->channels	= 2;
	profile->frames_per_pdu	= 6;
}

uint32_t aaf_nsr_to_hz(uint8_t nsr)
{
	switch (nsr) {
	case AVTP_AAF_PCM_NSR_8KHZ:	return 8000;
	case AVTP_AAF_PCM_NSR_16KHZ:	return 16000;
	case AVTP_AAF_PCM_NSR_24KHZ:	return 24000;
	case AVTP_AAF_PCM_NSR_32KHZ:	return 32000;
	case AVTP_AAF_PCM_NSR_44_1KHZ:	return 44100;
	case AVTP_AAF_PCM_NSR_48KHZ:	return 48000;
	case AVTP_AAF_PCM_NSR_88_2KHZ:	return 88200;
	case AVTP_AAF_PCM_NSR_96KHZ:	return 96000;
	case AVTP_AAF_PCM_NSR_176_4KHZ:	return 176400;
	case AVTP_AAF_PCM_NSR_192KHZ:	return 192000;
	default:			return 0;
	}
}

uint8_t aaf_format_to_sample_size(uint8_t format)
{
	switch (format) {
	case AVTP_AAF_FORMAT_INT_16BIT:		return 2;
	case AVTP_AAF_FORMAT_INT_24BIT:		return 3;
	case AVTP_AAF_FORMAT_INT_32BIT:		return 4;
	case AVTP_AAF_FORMAT_FLOAT_32BIT:	return 4;
	case AVTP_AAF_FORMAT_AES3_32BIT:	return 4;
	default:				return 0;
	}
}

void aaf_profile_describe(const struct aaf_profile *profile, char *buf,
			  size_t len)
{
	if (!profile || !buf || len == 0)
		return;

	snprintf(buf, len,
		 "stream 0x%016" PRIx64 "  %u Hz  %u ch  %u-bit  "
		 "%u frames/PDU  %.0f PDU/s  %zu byte payload",
		 profile->stream_id,
		 aaf_profile_sample_rate(profile),
		 profile->channels,
		 profile->bit_depth,
		 profile->frames_per_pdu,
		 aaf_profile_pdu_rate_hz(profile),
		 aaf_profile_data_len(profile));
}

const char *aaf_pdu_status_str(enum aaf_pdu_status status)
{
	switch (status) {
	case AAF_PDU_OK:		return "ok";
	case AAF_PDU_NOT_AAF:		return "not an AAF PDU";
	case AAF_PDU_OTHER_STREAM:	return "different stream";
	case AAF_PDU_MALFORMED:		return "malformed";
	case AAF_PDU_PROFILE:		return "profile mismatch";
	default:			return "unknown";
	}
}

/* Record a field disagreement. Under AAF_STRICT every one rejects; under
 * AAF_LENIENT only the checks the original is_valid_packet() performed
 * reject, so behaviour matches the pre-profile code exactly. */
static enum aaf_pdu_status note_mismatch(struct aaf_pdu_info *out,
					 const char *name, uint64_t got,
					 uint64_t want, bool legacy,
					 enum aaf_strictness mode)
{
	out->bad_field = name;
	out->bad_got = got;
	out->bad_want = want;

	if (legacy || mode == AAF_STRICT)
		return AAF_PDU_PROFILE;

	return AAF_PDU_OK;
}

enum aaf_pdu_status aaf_pdu_parse(const struct avtp_stream_pdu *pdu, size_t len,
				  const struct aaf_profile *profile,
				  enum aaf_strictness mode,
				  struct aaf_pdu_info *out)
{
	const struct avtp_common_pdu *common = (const struct avtp_common_pdu *)pdu;
	enum aaf_pdu_status status;
	uint64_t val64;
	uint32_t val32;
	size_t i;
	int res;

	if (!pdu || !profile || !out)
		return AAF_PDU_MALFORMED;

	memset(out, 0, sizeof(*out));

	if (profile->channels == 0 || profile->frames_per_pdu == 0 ||
	    aaf_profile_sample_size(profile) == 0)
		return AAF_PDU_MALFORMED;

	if (len != aaf_profile_pdu_size(profile))
		return AAF_PDU_MALFORMED;

	/* Subtype first: another subtype is not our traffic, and on a shared
	 * segment that is the common case. Never worth reporting. */
	res = avtp_pdu_get(common, AVTP_FIELD_SUBTYPE, &val32);
	if (res < 0)
		return AAF_PDU_MALFORMED;
	if (val32 != AVTP_SUBTYPE_AAF)
		return AAF_PDU_NOT_AAF;

	/* Stream ID next, for the same reason. */
	res = avtp_aaf_pdu_get(pdu, AVTP_AAF_FIELD_STREAM_ID, &val64);
	if (res < 0)
		return AAF_PDU_MALFORMED;
	if (val64 != profile->stream_id)
		return AAF_PDU_OTHER_STREAM;

	res = avtp_pdu_get(common, AVTP_FIELD_VERSION, &val32);
	if (res < 0)
		return AAF_PDU_MALFORMED;
	if (val32 != 0)
		return note_mismatch(out, "version", val32, 0, true, mode);

	/* legacy marks the checks the pre-profile is_valid_packet() performed.
	 * Those reject in both modes; the rest only under AAF_STRICT. */
	const struct {
		enum avtp_aaf_field field;
		uint64_t expect;
		const char *name;
		bool legacy;
	} checks[] = {
		{ AVTP_AAF_FIELD_TV, 1, "tv", true },
		{ AVTP_AAF_FIELD_FORMAT, profile->format, "format", true },
		{ AVTP_AAF_FIELD_NSR, profile->nsr, "nsr", true },
		{ AVTP_AAF_FIELD_CHAN_PER_FRAME, profile->channels,
		  "chan_per_frame", true },
		{ AVTP_AAF_FIELD_STREAM_DATA_LEN,
		  aaf_profile_data_len(profile), "stream_data_len", true },
		{ AVTP_AAF_FIELD_BIT_DEPTH, profile->bit_depth, "bit_depth",
		  false },
		{ AVTP_AAF_FIELD_SP, profile->sp, "sp", false },
	};

	for (i = 0; i < sizeof(checks) / sizeof(checks[0]); i++) {
		res = avtp_aaf_pdu_get(pdu, checks[i].field, &val64);
		if (res < 0)
			return AAF_PDU_MALFORMED;
		if (val64 != checks[i].expect) {
			status = note_mismatch(out, checks[i].name, val64,
					       checks[i].expect,
					       checks[i].legacy, mode);
			if (status != AAF_PDU_OK)
				return status;
		}
	}

	res = avtp_aaf_pdu_get(pdu, AVTP_AAF_FIELD_TIMESTAMP, &val64);
	if (res < 0)
		return AAF_PDU_MALFORMED;
	out->timestamp = (uint32_t)val64;

	res = avtp_aaf_pdu_get(pdu, AVTP_AAF_FIELD_SEQ_NUM, &val64);
	if (res < 0)
		return AAF_PDU_MALFORMED;
	out->seq_num = (uint8_t)val64;

	out->tv = true;
	out->payload = pdu->avtp_payload;
	out->payload_len = aaf_profile_data_len(profile);

	return AAF_PDU_OK;
}

int aaf_pdu_init(struct avtp_stream_pdu *pdu, const struct aaf_profile *profile)
{
	size_t i;
	int res;

	if (!pdu || !profile)
		return -1;

	res = avtp_aaf_pdu_init(pdu);
	if (res < 0)
		return -1;

	const struct {
		enum avtp_aaf_field field;
		uint64_t value;
	} fields[] = {
		{ AVTP_AAF_FIELD_TV, 1 },
		{ AVTP_AAF_FIELD_STREAM_ID, profile->stream_id },
		{ AVTP_AAF_FIELD_FORMAT, profile->format },
		{ AVTP_AAF_FIELD_NSR, profile->nsr },
		{ AVTP_AAF_FIELD_CHAN_PER_FRAME, profile->channels },
		{ AVTP_AAF_FIELD_BIT_DEPTH, profile->bit_depth },
		{ AVTP_AAF_FIELD_STREAM_DATA_LEN,
		  aaf_profile_data_len(profile) },
		{ AVTP_AAF_FIELD_SP, profile->sp },
	};

	for (i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
		res = avtp_aaf_pdu_set(pdu, fields[i].field, fields[i].value);
		if (res < 0)
			return -1;
	}

	return 0;
}
