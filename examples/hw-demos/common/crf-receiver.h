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

typedef void (*crf_ts_callback_fn)(uint64_t *timestamps, int count, uint8_t seq, void *ctx);
typedef void (*crf_drop_callback_fn)(uint8_t expected, uint8_t actual, void *ctx);

struct crf_receiver_config {
	char *ifname;
	uint8_t *macaddr;
	crf_ts_callback_fn on_timestamps;
	crf_drop_callback_fn on_drop;
	void *callback_ctx;
};

struct crf_receiver;

struct crf_receiver *crf_receiver_create(const struct crf_receiver_config *cfg);
struct crf_receiver *crf_receiver_create_from_fd(int fd,
						 crf_ts_callback_fn on_timestamps,
						 crf_drop_callback_fn on_drop,
						 void *ctx);
void crf_receiver_destroy(struct crf_receiver *rx);

int crf_receiver_fd(const struct crf_receiver *rx);

int crf_receiver_process(struct crf_receiver *rx);

uint64_t crf_receiver_wait_first(struct crf_receiver *rx, int timeout_sec);

uint64_t crf_receiver_pkt_count(const struct crf_receiver *rx);
uint64_t crf_receiver_drop_count(const struct crf_receiver *rx);
