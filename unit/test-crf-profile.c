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

/* CRF stream profile tests.
 *
 * Covers three things the hardware demos depend on:
 *
 *   - Derived rates. edge_hz and the servo's phase window follow from
 *     base_freq / timestamp_interval. These assertions pin the values that
 *     used to be the hardcoded PWM_FREQ_HZ and EDGE_TOLERANCE_NS macros, so a
 *     profile change that would silently retune the hardware fails here first.
 *
 *   - Wire compatibility. crf_pdu_init() must encode byte-for-byte what the
 *     existing CM4 talker emits.
 *
 *   - Validation. Every profile field is rejected on mismatch; mr and tu are
 *     surfaced rather than rejected.
 */

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>
#include <arpa/inet.h>
#include <string.h>

#include "avtp.h"
#include "avtp_crf.h"
#include "crf-profile.h"

/* Reference encoding produced by crf-talker-hw.c init_pdu() with the Table 28
 * profile. Captured from the working CM4 talker; any change to crf_pdu_init()
 * that alters the wire format will fail against this. */
static const uint8_t talker_header[] = {
	0x04, 0x80, 0x00, 0x01, 0xaa, 0xbb, 0xcc, 0xdd,
	0xee, 0xff, 0x00, 0x02, 0x00, 0x00, 0xbb, 0x80,
	0x00, 0x30, 0x00, 0xa0,
};

static void *make_pdu(const struct crf_profile *profile)
{
	void *pdu = test_calloc(1, crf_profile_pdu_size(profile));

	assert_non_null(pdu);
	assert_int_equal(crf_pdu_init(pdu, profile), 0);

	return pdu;
}

/*
 * Derived rates
 */

static void profile_defaults(void **state)
{
	struct crf_profile p;

	crf_profile_init(&p);

	assert_true(p.stream_id == 0xAABBCCDDEEFF0002);
	assert_int_equal(p.base_freq, 48000);
	assert_int_equal(p.timestamp_interval, 160);
	assert_int_equal(p.type, AVTP_CRF_TYPE_AUDIO_SAMPLE);
	assert_int_equal(p.pull, AVTP_CRF_PULL_MULT_BY_1);
	assert_int_equal(p.timestamps_per_pdu, 6);
}

static void profile_derived_edge_hz(void **state)
{
	struct crf_profile p;

	crf_profile_init(&p);

	/* Was the hardcoded PWM_FREQ_HZ / EDGE_FREQ_HZ / TARGET_GP0_HZ. */
	assert_true(crf_profile_media_hz(&p) == 48000.0);
	assert_true(crf_profile_edge_hz(&p) == 300.0);
}

static void profile_derived_tolerance(void **state)
{
	struct crf_profile p;

	crf_profile_init(&p);

	/* Was EDGE_TOLERANCE_NS = NSEC_PER_SEC / (300 * 2). */
	assert_true(crf_profile_edge_tolerance_ns(&p) == 1666666);
}

static void profile_derived_sizes(void **state)
{
	struct crf_profile p;

	crf_profile_init(&p);

	assert_int_equal(crf_profile_data_len(&p), 48);
	assert_int_equal(crf_profile_pdu_size(&p),
			 sizeof(struct avtp_crf_pdu) + 48);
}

static void profile_pull_scales_rate(void **state)
{
	struct crf_profile p;

	crf_profile_init(&p);
	p.pull = AVTP_CRF_PULL_MULT_BY_1_OVER_1_001;

	/* A pull-down stream is a different clock even though base_freq
	 * matches: roughly 1000 ppm slower. */
	assert_true(crf_profile_edge_hz(&p) < 300.0);
	assert_true(crf_profile_edge_hz(&p) > 299.6);
}

static void profile_pull_ratios(void **state)
{
	assert_true(crf_pull_ratio(AVTP_CRF_PULL_MULT_BY_1) == 1.0);
	assert_true(crf_pull_ratio(AVTP_CRF_PULL_MULT_BY_1_OVER_8) == 0.125);
	assert_true(crf_pull_ratio(AVTP_CRF_PULL_MULT_BY_24_OVER_25) < 1.0);
	assert_true(crf_pull_ratio(AVTP_CRF_PULL_MULT_BY_25_OVER_24) > 1.0);

	/* Unknown code must be reportable, not silently treated as 1.0. */
	assert_true(crf_pull_ratio(0x7f) == 0.0);
}

/*
 * Wire compatibility
 */

static void pdu_init_matches_talker(void **state)
{
	struct crf_profile p;
	void *pdu;

	crf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_memory_equal(pdu, talker_header, sizeof(talker_header));

	test_free(pdu);
}

static void pdu_init_null(void **state)
{
	struct crf_profile p;

	crf_profile_init(&p);

	assert_int_equal(crf_pdu_init(NULL, &p), -1);
}

/*
 * Validation
 */

static void parse_accepts_own_encoding(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	struct avtp_crf_pdu *pdu;
	int i;

	crf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(avtp_crf_pdu_set(pdu, AVTP_CRF_FIELD_SEQ_NUM, 42), 0);
	for (i = 0; i < 6; i++)
		pdu->crf_data[i] = htobe64(1000000000ULL + i * 3333333ULL);

	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p, CRF_STRICT, &info),
			 CRF_PDU_OK);
	assert_int_equal(info.count, 6);
	assert_int_equal(info.seq_num, 42);
	assert_true(info.timestamps[0] == 1000000000ULL);
	assert_true(info.timestamps[5] == 1000000000ULL + 5 * 3333333ULL);
	assert_false(info.mr);
	assert_false(info.tu);

	test_free(pdu);
}

static void parse_rejects_bad_length(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	void *pdu;
	size_t sz;

	crf_profile_init(&p);
	pdu = make_pdu(&p);
	sz = crf_profile_pdu_size(&p);

	assert_int_equal(crf_pdu_parse(pdu, sz - 1, &p, CRF_STRICT, &info),
			 CRF_PDU_MALFORMED);
	assert_int_equal(crf_pdu_parse(pdu, sz + 1, &p, CRF_STRICT, &info),
			 CRF_PDU_MALFORMED);
	assert_int_equal(crf_pdu_parse(pdu, 0, &p, CRF_STRICT, &info), CRF_PDU_MALFORMED);

	test_free(pdu);
}

static void parse_rejects_null(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;

	crf_profile_init(&p);

	assert_int_equal(crf_pdu_parse(NULL, 68, &p, CRF_STRICT, &info), CRF_PDU_MALFORMED);
}

static void parse_other_stream_is_quiet(void **state)
{
	struct crf_profile p, other;
	struct crf_pdu_info info;
	void *pdu;

	crf_profile_init(&p);
	pdu = make_pdu(&p);

	other = p;
	other.stream_id = 0xDEADBEEFCAFEBABEULL;

	/* Another talker's stream is normal traffic, not a misconfiguration. */
	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &other,
				       CRF_STRICT, &info),
			 CRF_PDU_OTHER_STREAM);

	test_free(pdu);
}

static void parse_not_crf_is_quiet(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	struct avtp_common_pdu *common;
	void *pdu;

	crf_profile_init(&p);
	pdu = make_pdu(&p);
	common = pdu;

	assert_int_equal(avtp_pdu_set(common, AVTP_FIELD_SUBTYPE,
				      AVTP_SUBTYPE_AAF), 0);
	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_STRICT, &info),
			 CRF_PDU_NOT_CRF);

	test_free(pdu);
}

/* Each of these is a field the reference listener either checked and we had
 * dropped, or nobody checked at all. */
static void assert_field_rejected(enum avtp_crf_field field, uint64_t bad,
				  const char *name, bool legacy)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	void *pdu;

	crf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(avtp_crf_pdu_set(pdu, field, bad), 0);

	/* Strict enforces every field. */
	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_STRICT, &info),
			 CRF_PDU_PROFILE);
	assert_non_null(info.bad_field);
	assert_string_equal(info.bad_field, name);
	assert_true(info.bad_got == bad);

	/* Lenient rejects only what the pre-profile validator rejected, but
	 * still reports the field either way. */
	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_LENIENT, &info),
			 legacy ? CRF_PDU_PROFILE : CRF_PDU_OK);
	assert_non_null(info.bad_field);
	assert_string_equal(info.bad_field, name);

	test_free(pdu);
}

static void parse_rejects_bad_fs(void **state)
{
	assert_field_rejected(AVTP_CRF_FIELD_FS, 1, "fs", false);
}

static void parse_rejects_bad_pull(void **state)
{
	/* base_freq still reads 48000; only pull reveals the real rate. */
	assert_field_rejected(AVTP_CRF_FIELD_PULL,
			      AVTP_CRF_PULL_MULT_BY_1_OVER_1_001, "pull",
			      false);
}

static void parse_rejects_bad_base_freq(void **state)
{
	assert_field_rejected(AVTP_CRF_FIELD_BASE_FREQ, 96000, "base_freq",
			      true);
}

static void parse_rejects_bad_timestamp_interval(void **state)
{
	/* 320 would be a valid CRF stream at 150 Hz, halving the edge rate
	 * and breaking the 1:1 CRF-to-edge invariant the hardware relies on. */
	assert_field_rejected(AVTP_CRF_FIELD_TIMESTAMP_INTERVAL, 320,
			      "timestamp_interval", false);
}

static void parse_rejects_bad_data_len(void **state)
{
	/* Previously clamped and accepted rather than rejected. */
	assert_field_rejected(AVTP_CRF_FIELD_CRF_DATA_LEN, 4096,
			      "crf_data_len", false);
}

static void parse_rejects_bad_type(void **state)
{
	assert_field_rejected(AVTP_CRF_FIELD_TYPE, AVTP_CRF_TYPE_VIDEO_FRAME,
			      "type", true);
}

static void parse_surfaces_mr_and_tu(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	void *pdu;

	crf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(avtp_crf_pdu_set(pdu, AVTP_CRF_FIELD_MR, 1), 0);
	assert_int_equal(avtp_crf_pdu_set(pdu, AVTP_CRF_FIELD_TU, 1), 0);

	/* Signalling, not validation: the PDU stays valid. */
	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_STRICT, &info),
			 CRF_PDU_OK);
	assert_true(info.mr);
	assert_true(info.tu);

	test_free(pdu);
}

static void parse_rejects_bad_timestamps_per_pdu(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	uint8_t buf[256] = { 0 };

	crf_profile_init(&p);

	p.timestamps_per_pdu = 0;
	assert_int_equal(crf_pdu_parse((void *)buf, sizeof(buf), &p, CRF_STRICT, &info),
			 CRF_PDU_MALFORMED);

	p.timestamps_per_pdu = CRF_MAX_TIMESTAMPS_PER_PDU + 1;
	assert_int_equal(crf_pdu_parse((void *)buf, sizeof(buf), &p, CRF_STRICT, &info),
			 CRF_PDU_MALFORMED);
}

/* Undersized crf_data_len must reject even under CRF_LENIENT: the old extract
 * path required enough bytes to fill the buffer, so accepting it here would be
 * more permissive than the code being replaced. */
static void parse_lenient_rejects_undersized_data_len(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	void *pdu;

	crf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(avtp_crf_pdu_set(pdu, AVTP_CRF_FIELD_CRF_DATA_LEN, 8),
			 0);

	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_LENIENT, &info),
			 CRF_PDU_PROFILE);
	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_STRICT, &info),
			 CRF_PDU_PROFILE);

	test_free(pdu);
}

/* A conformant PDU leaves bad_field NULL, so the receiver can distinguish
 * "accepted and clean" from "accepted but non-conformant". */
static void parse_conformant_sets_no_bad_field(void **state)
{
	struct crf_profile p;
	struct crf_pdu_info info;
	void *pdu;

	crf_profile_init(&p);
	pdu = make_pdu(&p);

	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_LENIENT, &info),
			 CRF_PDU_OK);
	assert_null(info.bad_field);

	assert_int_equal(crf_pdu_parse(pdu, crf_profile_pdu_size(&p), &p,
				       CRF_STRICT, &info),
			 CRF_PDU_OK);
	assert_null(info.bad_field);

	test_free(pdu);
}

static void status_strings_present(void **state)
{
	assert_string_equal(crf_pdu_status_str(CRF_PDU_OK), "ok");
	assert_non_null(crf_pdu_status_str(CRF_PDU_NOT_CRF));
	assert_non_null(crf_pdu_status_str(CRF_PDU_OTHER_STREAM));
	assert_non_null(crf_pdu_status_str(CRF_PDU_MALFORMED));
	assert_non_null(crf_pdu_status_str(CRF_PDU_PROFILE));
}

int main(void)
{
	const struct CMUnitTest tests[] = {
		cmocka_unit_test(profile_defaults),
		cmocka_unit_test(profile_derived_edge_hz),
		cmocka_unit_test(profile_derived_tolerance),
		cmocka_unit_test(profile_derived_sizes),
		cmocka_unit_test(profile_pull_scales_rate),
		cmocka_unit_test(profile_pull_ratios),
		cmocka_unit_test(pdu_init_matches_talker),
		cmocka_unit_test(pdu_init_null),
		cmocka_unit_test(parse_accepts_own_encoding),
		cmocka_unit_test(parse_rejects_bad_length),
		cmocka_unit_test(parse_rejects_null),
		cmocka_unit_test(parse_other_stream_is_quiet),
		cmocka_unit_test(parse_not_crf_is_quiet),
		cmocka_unit_test(parse_rejects_bad_fs),
		cmocka_unit_test(parse_rejects_bad_pull),
		cmocka_unit_test(parse_rejects_bad_base_freq),
		cmocka_unit_test(parse_rejects_bad_timestamp_interval),
		cmocka_unit_test(parse_rejects_bad_data_len),
		cmocka_unit_test(parse_rejects_bad_type),
		cmocka_unit_test(parse_surfaces_mr_and_tu),
		cmocka_unit_test(parse_rejects_bad_timestamps_per_pdu),
		cmocka_unit_test(parse_lenient_rejects_undersized_data_len),
		cmocka_unit_test(parse_conformant_sets_no_bad_field),
		cmocka_unit_test(status_strings_present),
	};

	return cmocka_run_group_tests(tests, NULL, NULL);
}
