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

/* AAF stream profile tests.
 *
 * The derived-size assertions matter most: after the profile refactor, every
 * buffer in the AAF talker and listener is sized from data_len, pdu_size or
 * samples_per_pdu. These pin the values the old NUM_CHANNELS/SAMPLE_SIZE/
 * FRAMES_PER_PDU macros produced, so a profile change that would silently
 * resize an ALSA buffer or a byteswap loop fails here first.
 */

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <arpa/inet.h>
#include <string.h>

#include "avtp.h"
#include "avtp_aaf.h"
#include "aaf-profile.h"

/* Reference encoding captured from the pre-refactor aaf-talker-hw.c,
 * before init_pdu() was replaced. Any change to aaf_pdu_init() that
 * alters the wire format fails against this. */
static const uint8_t talker_header[] = {
	0x02, 0x81, 0x00, 0x00, 0xaa, 0xbb, 0xcc, 0xdd,
	0xee, 0xff, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x50, 0x02, 0x20, 0x00, 0x30, 0x00, 0x00,
};

static void *make_pdu(const struct aaf_profile *profile)
{
	void *pdu = test_calloc(1, aaf_profile_pdu_size(profile));

	assert_non_null(pdu);
	assert_int_equal(aaf_pdu_init(pdu, profile), 0);

	return pdu;
}

/*
 * Defaults and derived sizes
 */

static void profile_defaults(void **state)
{
	struct aaf_profile p;

	aaf_profile_init(&p);

	assert_true(p.stream_id == 0xAABBCCDDEEFF0002);
	assert_int_equal(p.nsr, AVTP_AAF_PCM_NSR_48KHZ);
	assert_int_equal(p.format, AVTP_AAF_FORMAT_INT_32BIT);
	assert_int_equal(p.bit_depth, 32);
	assert_int_equal(p.sp, AVTP_AAF_PCM_SP_NORMAL);
	assert_int_equal(p.channels, 2);
	assert_int_equal(p.frames_per_pdu, 6);
}

static void profile_derived_sizes(void **state)
{
	struct aaf_profile p;

	aaf_profile_init(&p);

	/* Were SAMPLE_SIZE, DATA_LEN, PDU_SIZE and FRAMES_PER_PDU *
	 * NUM_CHANNELS. Every buffer in the talker and listener is sized
	 * from these. */
	assert_int_equal(aaf_profile_sample_size(&p), 4);
	assert_int_equal(aaf_profile_sample_rate(&p), 48000);
	assert_int_equal(aaf_profile_data_len(&p), 48);
	assert_int_equal(aaf_profile_pdu_size(&p),
			 sizeof(struct avtp_stream_pdu) + 48);
	assert_int_equal(aaf_profile_samples_per_pdu(&p), 12);
}

static void profile_derived_pdu_rate(void **state)
{
	struct aaf_profile p;

	aaf_profile_init(&p);

	/* 48000 / 6 = 8000 PDU/s, the cadence the listener's gate assumes. */
	assert_true(aaf_profile_pdu_rate_hz(&p) == 8000.0);
}

static void nsr_table(void **state)
{
	assert_int_equal(aaf_nsr_to_hz(AVTP_AAF_PCM_NSR_8KHZ), 8000);
	assert_int_equal(aaf_nsr_to_hz(AVTP_AAF_PCM_NSR_44_1KHZ), 44100);
	assert_int_equal(aaf_nsr_to_hz(AVTP_AAF_PCM_NSR_48KHZ), 48000);
	assert_int_equal(aaf_nsr_to_hz(AVTP_AAF_PCM_NSR_96KHZ), 96000);
	assert_int_equal(aaf_nsr_to_hz(AVTP_AAF_PCM_NSR_192KHZ), 192000);

	/* Unrecognised must be reportable, not silently treated as 48 kHz. */
	assert_int_equal(aaf_nsr_to_hz(AVTP_AAF_PCM_NSR_USER), 0);
	assert_int_equal(aaf_nsr_to_hz(0x7f), 0);
}

static void format_table(void **state)
{
	assert_int_equal(aaf_format_to_sample_size(AVTP_AAF_FORMAT_INT_16BIT), 2);
	assert_int_equal(aaf_format_to_sample_size(AVTP_AAF_FORMAT_INT_24BIT), 3);
	assert_int_equal(aaf_format_to_sample_size(AVTP_AAF_FORMAT_INT_32BIT), 4);
	assert_int_equal(aaf_format_to_sample_size(AVTP_AAF_FORMAT_FLOAT_32BIT), 4);
	assert_int_equal(aaf_format_to_sample_size(AVTP_AAF_FORMAT_USER), 0);
}

/* A different profile must resize everything coherently. */
static void profile_resizes_coherently(void **state)
{
	struct aaf_profile p;

	aaf_profile_init(&p);
	p.channels = 8;
	p.frames_per_pdu = 12;
	p.format = AVTP_AAF_FORMAT_INT_16BIT;
	p.nsr = AVTP_AAF_PCM_NSR_96KHZ;

	assert_int_equal(aaf_profile_sample_size(&p), 2);
	assert_int_equal(aaf_profile_sample_rate(&p), 96000);
	assert_int_equal(aaf_profile_data_len(&p), 2 * 8 * 12);
	assert_int_equal(aaf_profile_samples_per_pdu(&p), 96);
	assert_true(aaf_profile_pdu_rate_hz(&p) == 8000.0);
}

/*
 * Wire compatibility
 */

static void pdu_init_matches_talker(void **state)
{
	struct aaf_profile p;
	void *pdu;

	aaf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_memory_equal(pdu, talker_header, sizeof(talker_header));

	test_free(pdu);
}

static void pdu_init_null(void **state)
{
	struct aaf_profile p;

	aaf_profile_init(&p);

	assert_int_equal(aaf_pdu_init(NULL, &p), -1);
}

/*
 * Validation
 */

static void parse_accepts_own_encoding(void **state)
{
	struct aaf_profile p;
	struct aaf_pdu_info info;
	struct avtp_stream_pdu *pdu;

	aaf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_SEQ_NUM, 77), 0);
	assert_int_equal(avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_TIMESTAMP,
					  0xDEADBEEF), 0);

	assert_int_equal(aaf_pdu_parse(pdu, aaf_profile_pdu_size(&p), &p,
				       AAF_STRICT, &info),
			 AAF_PDU_OK);
	assert_int_equal(info.seq_num, 77);
	assert_true(info.timestamp == 0xDEADBEEF);
	assert_true(info.tv);
	assert_int_equal(info.payload_len, 48);
	assert_ptr_equal(info.payload, pdu->avtp_payload);
	assert_null(info.bad_field);

	test_free(pdu);
}

static void parse_rejects_bad_length(void **state)
{
	struct aaf_profile p;
	struct aaf_pdu_info info;
	void *pdu;
	size_t sz;

	aaf_profile_init(&p);
	pdu = make_pdu(&p);
	sz = aaf_profile_pdu_size(&p);

	assert_int_equal(aaf_pdu_parse(pdu, sz - 1, &p, AAF_STRICT, &info),
			 AAF_PDU_MALFORMED);
	assert_int_equal(aaf_pdu_parse(pdu, sz + 1, &p, AAF_STRICT, &info),
			 AAF_PDU_MALFORMED);
	assert_int_equal(aaf_pdu_parse(pdu, 0, &p, AAF_STRICT, &info),
			 AAF_PDU_MALFORMED);

	test_free(pdu);
}

static void parse_rejects_null(void **state)
{
	struct aaf_profile p;
	struct aaf_pdu_info info;

	aaf_profile_init(&p);

	assert_int_equal(aaf_pdu_parse(NULL, 72, &p, AAF_STRICT, &info),
			 AAF_PDU_MALFORMED);
}

/* A profile that cannot describe a PDU must be rejected, not divided by. */
static void parse_rejects_degenerate_profile(void **state)
{
	struct aaf_profile p;
	struct aaf_pdu_info info;
	uint8_t buf[256] = { 0 };

	aaf_profile_init(&p);
	p.channels = 0;
	assert_int_equal(aaf_pdu_parse((void *)buf, sizeof(buf), &p,
				       AAF_STRICT, &info),
			 AAF_PDU_MALFORMED);

	aaf_profile_init(&p);
	p.frames_per_pdu = 0;
	assert_int_equal(aaf_pdu_parse((void *)buf, sizeof(buf), &p,
				       AAF_STRICT, &info),
			 AAF_PDU_MALFORMED);

	aaf_profile_init(&p);
	p.format = AVTP_AAF_FORMAT_USER;	/* sample size 0 */
	assert_int_equal(aaf_pdu_parse((void *)buf, sizeof(buf), &p,
				       AAF_STRICT, &info),
			 AAF_PDU_MALFORMED);
}

static void parse_other_stream_is_quiet(void **state)
{
	struct aaf_profile p, other;
	struct aaf_pdu_info info;
	void *pdu;

	aaf_profile_init(&p);
	pdu = make_pdu(&p);

	other = p;
	other.stream_id = 0xDEADBEEFCAFEBABEULL;

	assert_int_equal(aaf_pdu_parse(pdu, aaf_profile_pdu_size(&p), &other,
				       AAF_STRICT, &info),
			 AAF_PDU_OTHER_STREAM);

	test_free(pdu);
}

static void parse_not_aaf_is_quiet(void **state)
{
	struct aaf_profile p;
	struct aaf_pdu_info info;
	void *pdu;

	aaf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(avtp_pdu_set(pdu, AVTP_FIELD_SUBTYPE,
				      AVTP_SUBTYPE_CRF), 0);
	assert_int_equal(aaf_pdu_parse(pdu, aaf_profile_pdu_size(&p), &p,
				       AAF_STRICT, &info),
			 AAF_PDU_NOT_AAF);

	test_free(pdu);
}

/* legacy == true means the pre-profile is_valid_packet() also rejected it,
 * so lenient mode must reject it too. */
static void assert_field_rejected(enum avtp_aaf_field field, uint64_t bad,
				  const char *name, bool legacy)
{
	struct aaf_profile p;
	struct aaf_pdu_info info;
	void *pdu;

	aaf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(avtp_aaf_pdu_set(pdu, field, bad), 0);

	assert_int_equal(aaf_pdu_parse(pdu, aaf_profile_pdu_size(&p), &p,
				       AAF_STRICT, &info),
			 AAF_PDU_PROFILE);
	assert_non_null(info.bad_field);
	assert_string_equal(info.bad_field, name);
	assert_true(info.bad_got == bad);

	assert_int_equal(aaf_pdu_parse(pdu, aaf_profile_pdu_size(&p), &p,
				       AAF_LENIENT, &info),
			 legacy ? AAF_PDU_PROFILE : AAF_PDU_OK);
	assert_non_null(info.bad_field);
	assert_string_equal(info.bad_field, name);

	test_free(pdu);
}

static void parse_rejects_bad_tv(void **state)
{
	assert_field_rejected(AVTP_AAF_FIELD_TV, 0, "tv", true);
}

static void parse_rejects_bad_format(void **state)
{
	assert_field_rejected(AVTP_AAF_FIELD_FORMAT,
			      AVTP_AAF_FORMAT_INT_16BIT, "format", true);
}

static void parse_rejects_bad_nsr(void **state)
{
	assert_field_rejected(AVTP_AAF_FIELD_NSR,
			      AVTP_AAF_PCM_NSR_96KHZ, "nsr", true);
}

static void parse_rejects_bad_channels(void **state)
{
	assert_field_rejected(AVTP_AAF_FIELD_CHAN_PER_FRAME, 4,
			      "chan_per_frame", true);
}

static void parse_rejects_bad_data_len(void **state)
{
	assert_field_rejected(AVTP_AAF_FIELD_STREAM_DATA_LEN, 96,
			      "stream_data_len", true);
}

/* bit_depth and sp are the two the previous validator never inspected. A
 * 24-bit stream in 32-bit containers has the same format code and the same
 * data length as a 32-bit stream; only bit_depth separates them. */
static void parse_rejects_bad_bit_depth(void **state)
{
	assert_field_rejected(AVTP_AAF_FIELD_BIT_DEPTH, 24, "bit_depth", false);
}

/* Sparse mode changes which PDUs carry a valid timestamp, which the
 * presentation-time gate assumes is all of them. */
static void parse_rejects_bad_sp(void **state)
{
	assert_field_rejected(AVTP_AAF_FIELD_SP, AVTP_AAF_PCM_SP_SPARSE, "sp",
			      false);
}

static void parse_conformant_sets_no_bad_field(void **state)
{
	struct aaf_profile p;
	struct aaf_pdu_info info;
	void *pdu;

	aaf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(aaf_pdu_parse(pdu, aaf_profile_pdu_size(&p), &p,
				       AAF_LENIENT, &info),
			 AAF_PDU_OK);
	assert_null(info.bad_field);

	assert_int_equal(aaf_pdu_parse(pdu, aaf_profile_pdu_size(&p), &p,
				       AAF_STRICT, &info),
			 AAF_PDU_OK);
	assert_null(info.bad_field);

	test_free(pdu);
}

static void status_strings_present(void **state)
{
	assert_string_equal(aaf_pdu_status_str(AAF_PDU_OK), "ok");
	assert_non_null(aaf_pdu_status_str(AAF_PDU_NOT_AAF));
	assert_non_null(aaf_pdu_status_str(AAF_PDU_OTHER_STREAM));
	assert_non_null(aaf_pdu_status_str(AAF_PDU_MALFORMED));
	assert_non_null(aaf_pdu_status_str(AAF_PDU_PROFILE));
}

int main(void)
{
	const struct CMUnitTest tests[] = {
		cmocka_unit_test(profile_defaults),
		cmocka_unit_test(profile_derived_sizes),
		cmocka_unit_test(profile_derived_pdu_rate),
		cmocka_unit_test(nsr_table),
		cmocka_unit_test(format_table),
		cmocka_unit_test(profile_resizes_coherently),
		cmocka_unit_test(pdu_init_matches_talker),
		cmocka_unit_test(pdu_init_null),
		cmocka_unit_test(parse_accepts_own_encoding),
		cmocka_unit_test(parse_rejects_bad_length),
		cmocka_unit_test(parse_rejects_null),
		cmocka_unit_test(parse_rejects_degenerate_profile),
		cmocka_unit_test(parse_other_stream_is_quiet),
		cmocka_unit_test(parse_not_aaf_is_quiet),
		cmocka_unit_test(parse_rejects_bad_tv),
		cmocka_unit_test(parse_rejects_bad_format),
		cmocka_unit_test(parse_rejects_bad_nsr),
		cmocka_unit_test(parse_rejects_bad_channels),
		cmocka_unit_test(parse_rejects_bad_data_len),
		cmocka_unit_test(parse_rejects_bad_bit_depth),
		cmocka_unit_test(parse_rejects_bad_sp),
		cmocka_unit_test(parse_conformant_sets_no_bad_field),
		cmocka_unit_test(status_strings_present),
	};

	return cmocka_run_group_tests(tests, NULL, NULL);
}
