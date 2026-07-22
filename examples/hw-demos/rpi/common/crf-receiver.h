/*
 * CRF Receiver Module
 *
 * Reusable CRF packet reception with callback-based timestamp delivery.
 * Handles socket lifecycle, packet validation, sequence tracking, and
 * drop detection. Consumer provides callbacks for timestamp processing.
 *
 * Exposes socket fd for integration into caller's select/poll loop.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <linux/if.h>
#include <linux/if_ether.h>

#include "crf-profile.h"

typedef void (*crf_ts_callback_fn)(uint64_t *timestamps, int count, uint8_t seq, void *ctx);
typedef void (*crf_drop_callback_fn)(uint8_t expected, uint8_t actual, void *ctx);

struct crf_receiver_config {
	char *ifname;
	uint8_t *macaddr;
	crf_ts_callback_fn on_timestamps;
	crf_drop_callback_fn on_drop;
	void *callback_ctx;

	/* Stream profile to validate against. The caller owns it and derives
	 * its hardware rates from the same struct, so the edge rate the clock
	 * backend generates and the stream the receiver accepts cannot drift
	 * apart. NULL selects the Table 28 defaults. */
	const struct crf_profile *profile;
};

struct crf_receiver;

struct crf_receiver *crf_receiver_create(const struct crf_receiver_config *cfg);
struct crf_receiver *crf_receiver_create_from_fd(int fd,
						 crf_ts_callback_fn on_timestamps,
						 crf_drop_callback_fn on_drop,
						 void *ctx);
void crf_receiver_destroy(struct crf_receiver *rx);

/* Select how much of the CRF profile to enforce. Receivers start lenient,
 * which reproduces the pre-profile validator exactly; non-conformant fields
 * are reported once on stderr but still delivered. Enable strict once a
 * lenient run has shown the talker is conformant. */
void crf_receiver_set_strict(struct crf_receiver *rx, bool strict);

/* The profile this receiver validates against. Callers derive the feedback
 * edge rate and servo phase window from it. */
const struct crf_profile *crf_receiver_profile(const struct crf_receiver *rx);

int crf_receiver_fd(const struct crf_receiver *rx);

int crf_receiver_process(struct crf_receiver *rx);

uint64_t crf_receiver_wait_first(struct crf_receiver *rx, int timeout_sec);

uint64_t crf_receiver_pkt_count(const struct crf_receiver *rx);
uint64_t crf_receiver_drop_count(const struct crf_receiver *rx);
