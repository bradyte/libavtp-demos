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

/* CRF Stream Profile — IEEE 1722-2016 Clause 10, Table 28
 *
 * One runtime definition of the CRF wire format, shared by talker and
 * listener. The talker encodes it; the listener validates against it. Every
 * dependent rate is derived from it, so nothing downstream hardcodes a
 * frequency:
 *
 *     edge_hz = base_freq x pull / timestamp_interval
 *             = 48000 x 1 / 160
 *             = 300 Hz          <- one feedback edge per CRF timestamp
 *
 * That 300 Hz is what the media clock backend must generate on its GPIO
 * (BCM2711 PWM, or RP1 clk_gp0), and half its period is the phase window the
 * servo accepts before declaring loss of sync. Both follow from the profile.
 *
 * Ownership: the application holds the struct. Call crf_profile_init() for the
 * Table 28 defaults, apply any command-line overrides, then pass it to
 * crf_pdu_init() (talker) or crf_pdu_parse() (listener).
 *
 *
 * NOTE — planned follow-on: network-owned profiles.
 *
 * The profile is currently application-owned: defaults plus CLI overrides. A
 * listener could instead discover it from the first PDU carrying the expected
 * stream_id, letting one binary lock to any conformant talker rather than only
 * a preconfigured one.
 *
 * The parse path below needs no change for that — only the source of the
 * struct changes. Three things must land with it:
 *
 *   1. Backend capability check. A discovered edge_hz must be offered to the
 *      media clock backend (PWM divider range, CS2600 ratio and its Q20.12 /
 *      Q12.20 format selection, RP1 clk_gp0 divider) before it is accepted, so
 *      an unsupportable stream fails at startup with a specific message rather
 *      than running a servo that can never lock.
 *
 *   2. Freeze after discovery. Once the servo is running and ALSA is open, the
 *      profile must be locked and later PDUs validated against the locked
 *      values. A mid-stream parameter change is a reconfiguration event, not
 *      something to absorb silently.
 *
 *   3. Cross-stream agreement. CRF base_freq and AAF nsr describe the same
 *      clock. While both are hardcoded they cannot disagree; once both are
 *      discovered independently, nothing catches a 48 kHz CRF stream feeding a
 *      96 kHz AAF stream. That assertion has to be explicit.
 *
 * Discovery also inverts the CM4 bring-up order: PWM and CS2600 are configured
 * before the socket opens today, including a PWM enable aligned to a PTP second
 * boundary. Discovery needs a packet first, which moves the most
 * timing-sensitive sequence in the demo. Worth its own hardware validation
 * pass, separate from this one.
 */

#pragma once

#include <endian.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avtp_crf.h"

/* Upper bound on timestamps carried in one CRF PDU. Sizes the extraction
 * buffer; the profile's timestamps_per_pdu selects how many are used. */
#define CRF_MAX_TIMESTAMPS_PER_PDU	6

#define NSEC_PER_SEC			1000000000ULL

/* The CRF wire format. A talker emits exactly these values; a listener drops
 * any PDU on its stream_id that disagrees. */
struct crf_profile {
	uint64_t stream_id;
	uint32_t base_freq;		/* Hz, AVTP_CRF_FIELD_BASE_FREQ */
	uint16_t timestamp_interval;	/* media clock ticks per timestamp */
	uint8_t  type;			/* AVTP_CRF_TYPE_* */
	uint8_t  pull;			/* AVTP_CRF_PULL_* */
	uint8_t  timestamps_per_pdu;
};

/* Populate with the IEEE 1722-2016 Table 28 recommendation: 48 kHz audio
 * sample clock, timestamp every 160 ticks, 6 timestamps per PDU. */
void crf_profile_init(struct crf_profile *profile);

/* Sample rate multiplier encoded by AVTP_CRF_FIELD_PULL. Returns 0.0 for an
 * unrecognised code so callers can reject it. */
double crf_pull_ratio(uint8_t pull);

/* Human-readable one-liner for startup banners. Writes at most len bytes. */
void crf_profile_describe(const struct crf_profile *profile, char *buf,
			  size_t len);

/* Wire size of one PDU under this profile. */
static inline size_t crf_profile_data_len(const struct crf_profile *profile)
{
	return (size_t)profile->timestamps_per_pdu * sizeof(uint64_t);
}

static inline size_t crf_profile_pdu_size(const struct crf_profile *profile)
{
	return sizeof(struct avtp_crf_pdu) + crf_profile_data_len(profile);
}

/* Effective media clock rate in Hz, including the pull multiplier. */
static inline double crf_profile_media_hz(const struct crf_profile *profile)
{
	return profile->base_freq * crf_pull_ratio(profile->pull);
}

/* Feedback edge rate: one edge per CRF timestamp. This is the frequency the
 * media clock backend must produce. */
static inline double crf_profile_edge_hz(const struct crf_profile *profile)
{
	return crf_profile_media_hz(profile) / profile->timestamp_interval;
}

/* Half an edge period — the phase error window the servo tolerates before
 * treating the edge as unrelated to the timestamp and resynchronising. */
static inline int64_t
crf_profile_edge_tolerance_ns(const struct crf_profile *profile)
{
	return (int64_t)(NSEC_PER_SEC / (2.0 * crf_profile_edge_hz(profile)));
}

enum crf_pdu_status {
	CRF_PDU_OK = 0,
	CRF_PDU_NOT_CRF,	/* another subtype: not our traffic, drop quiet */
	CRF_PDU_OTHER_STREAM,	/* another stream_id: drop quiet */
	CRF_PDU_MALFORMED,	/* field read failed, or length disagrees */
	CRF_PDU_PROFILE,	/* our stream, wrong parameters: report once */
};

/* How much of the profile to enforce.
 *
 * CRF_LENIENT reproduces exactly what the pre-profile validator enforced:
 * subtype, version, sv, stream_id, type, base_freq, and crf_data_len large
 * enough to fill the PDU. The fields that validator never inspected — fs,
 * pull, timestamp_interval, and an over-long crf_data_len — are reported
 * through bad_field but do not reject the PDU. Behaviour is bit-identical to
 * the original path on every input, which makes it safe to deploy on a
 * working rig while the reports are observed.
 *
 * CRF_STRICT enforces every field. Adopt it once the logs from a lenient run
 * show the talker is conformant. */
enum crf_strictness {
	CRF_LENIENT = 0,
	CRF_STRICT,
};

const char *crf_pdu_status_str(enum crf_pdu_status status);

/* Everything a listener needs from one PDU. */
struct crf_pdu_info {
	uint64_t timestamps[CRF_MAX_TIMESTAMPS_PER_PDU];
	uint8_t  count;
	uint8_t  seq_num;

	/* Signalling fields. Surfaced, never grounds for rejection.
	 *   mr - media clock restart toggle. When it flips, the talker has
	 *        restarted its media clock: drop sync and let the servo
	 *        reacquire rather than chasing a step.
	 *   tu - timestamps uncertain. The talker's own clock source is not
	 *        locked; hold the servo instead of chasing values the talker
	 *        does not trust. */
	bool mr;
	bool tu;

	/* Which field disagreed with the profile, and what it held.
	 *
	 * Set when status == CRF_PDU_PROFILE. Also set when status ==
	 * CRF_PDU_OK under CRF_LENIENT, meaning the PDU was accepted for
	 * compatibility but is not conformant — worth logging once. NULL on a
	 * fully conformant PDU. */
	const char *bad_field;
	uint64_t bad_got;
	uint64_t bad_want;
};

/* Validate the profile fields and extract the payload in a single pass.
 * @len must be the number of bytes actually received. See enum crf_strictness
 * for what @mode does and does not enforce. */
enum crf_pdu_status crf_pdu_parse(const struct avtp_crf_pdu *pdu, size_t len,
				  const struct crf_profile *profile,
				  enum crf_strictness mode,
				  struct crf_pdu_info *out);

/* Encode the profile into a PDU header. Talker side; sequence number and
 * timestamps are filled per packet by the caller. */
int crf_pdu_init(struct avtp_crf_pdu *pdu, const struct crf_profile *profile);
