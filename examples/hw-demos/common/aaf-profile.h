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

/* AAF Stream Profile — IEEE 1722-2016 Clause 7, PCM format
 *
 * One runtime definition of the AAF wire format, shared by talker and
 * listener. Everything the ALSA device and the packet layout need derives
 * from it, so nothing hardcodes a channel count or sample size:
 *
 *     sample_size = f(format)          INT_32BIT  -> 4 bytes
 *     sample_rate = f(nsr)             NSR_48KHZ  -> 48000 Hz
 *     data_len    = sample_size x channels x frames_per_pdu
 *     pdu_size    = sizeof(avtp_stream_pdu) + data_len
 *
 * nsr and format are the wire encodings and the source of truth; the rate
 * and byte width are computed from them rather than stored alongside, so
 * they cannot disagree.
 *
 * Ownership matches crf-profile.h: the application holds the struct, calls
 * aaf_profile_init() for the defaults, applies any overrides, then passes it
 * to aaf_pdu_init() (talker) or aaf_pdu_parse() (listener).
 *
 *
 * NOTE — the CRF/AAF agreement this does not yet enforce.
 *
 * A CRF stream's base_freq and an AAF stream's nsr describe the same media
 * clock. While both were hardcoded they could not disagree. Now that each is
 * a runtime value, a 48 kHz recovered clock feeding a 96 kHz AAF stream would
 * pass every check in both profiles independently.
 *
 * The assertion belongs wherever a binary holds both structs:
 *
 *     crf_profile_media_hz(&crf) == aaf_profile_sample_rate(&aaf)
 *
 * No current binary holds both — crf-listener-hw does clock recovery and
 * aaf-listener-hw does playback — so there is nowhere to put it yet. It
 * becomes necessary as soon as the two are combined, or when either profile
 * is discovered from the network rather than supplied by the application.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avtp_aaf.h"

/* The AAF wire format. A talker emits exactly these values; a listener drops
 * any PDU on its stream_id that disagrees. */
struct aaf_profile {
	uint64_t stream_id;
	uint8_t  nsr;			/* AVTP_AAF_PCM_NSR_*   */
	uint8_t  format;		/* AVTP_AAF_FORMAT_*    */
	uint8_t  bit_depth;		/* significant bits per sample */
	uint8_t  sp;			/* AVTP_AAF_PCM_SP_*    */
	uint8_t  channels;
	uint16_t frames_per_pdu;
};

/* Populate with the profile the hw-demos use: 48 kHz, stereo, 32-bit integer
 * samples, 6 frames per PDU (8000 packets/s), normal sparse mode. */
void aaf_profile_init(struct aaf_profile *profile);

/* Sample rate in Hz for an AVTP_AAF_PCM_NSR_* code, or 0 if unrecognised. */
uint32_t aaf_nsr_to_hz(uint8_t nsr);

/* Bytes per sample for an AVTP_AAF_FORMAT_* code, or 0 if unrecognised. */
uint8_t aaf_format_to_sample_size(uint8_t format);

/* Human-readable one-liner for startup banners. Writes at most len bytes. */
void aaf_profile_describe(const struct aaf_profile *profile, char *buf,
			  size_t len);

static inline uint32_t aaf_profile_sample_rate(const struct aaf_profile *p)
{
	return aaf_nsr_to_hz(p->nsr);
}

static inline uint8_t aaf_profile_sample_size(const struct aaf_profile *p)
{
	return aaf_format_to_sample_size(p->format);
}

/* Payload bytes carried by one PDU. */
static inline size_t aaf_profile_data_len(const struct aaf_profile *p)
{
	return (size_t)aaf_profile_sample_size(p) * p->channels *
	       p->frames_per_pdu;
}

static inline size_t aaf_profile_pdu_size(const struct aaf_profile *p)
{
	return sizeof(struct avtp_stream_pdu) + aaf_profile_data_len(p);
}

/* Samples per PDU across all channels — the interleaved buffer length. */
static inline size_t aaf_profile_samples_per_pdu(const struct aaf_profile *p)
{
	return (size_t)p->channels * p->frames_per_pdu;
}

/* Packet rate implied by the profile, in PDUs per second. */
static inline double aaf_profile_pdu_rate_hz(const struct aaf_profile *p)
{
	return (double)aaf_profile_sample_rate(p) / p->frames_per_pdu;
}

enum aaf_pdu_status {
	AAF_PDU_OK = 0,
	AAF_PDU_NOT_AAF,	/* another subtype: not our traffic, drop quiet */
	AAF_PDU_OTHER_STREAM,	/* another stream_id: drop quiet */
	AAF_PDU_MALFORMED,	/* field read failed, or length disagrees */
	AAF_PDU_PROFILE,	/* our stream, wrong parameters: report once */
};

const char *aaf_pdu_status_str(enum aaf_pdu_status status);

/* How much of the profile to enforce. Mirrors enum crf_strictness.
 *
 * AAF_LENIENT enforces exactly what the previous is_valid_packet() enforced:
 * subtype, version, tv, stream_id, format, nsr, channels and
 * stream_data_len. The fields it never inspected — bit_depth and sp — are
 * reported through bad_field but do not reject the PDU, so behaviour on a
 * working rig is unchanged.
 *
 * AAF_STRICT enforces every field. bit_depth matters because a 24-bit stream
 * in 32-bit containers is silently misread as full scale; sp matters because
 * sparse mode changes which packets carry a valid timestamp, which the
 * presentation-time gate depends on. */
enum aaf_strictness {
	AAF_LENIENT = 0,
	AAF_STRICT,
};

/* Everything a listener needs from one PDU. */
struct aaf_pdu_info {
	const void *payload;	/* interleaved samples, network byte order */
	size_t   payload_len;
	uint32_t timestamp;	/* AVTP presentation time, lower 32 bits */
	uint8_t  seq_num;
	bool     tv;		/* timestamp valid */

	/* Set when status == AAF_PDU_PROFILE, and also when status == AAF_PDU_OK
	 * under AAF_LENIENT to flag an accepted but non-conformant PDU. */
	const char *bad_field;
	uint64_t bad_got;
	uint64_t bad_want;
};

/* Validate the profile fields and locate the payload in a single pass.
 * @len must be the number of bytes actually received. */
enum aaf_pdu_status aaf_pdu_parse(const struct avtp_stream_pdu *pdu, size_t len,
				  const struct aaf_profile *profile,
				  enum aaf_strictness mode,
				  struct aaf_pdu_info *out);

/* Encode the profile into a PDU header. Talker side; sequence number,
 * timestamp and payload are filled per packet by the caller. */
int aaf_pdu_init(struct avtp_stream_pdu *pdu, const struct aaf_profile *profile);
