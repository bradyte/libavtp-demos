/*
 * AF_XDP (XSK) receive utility for AVTP listeners.
 * Raw kernel syscall implementation — no external dependencies.
 */

#ifndef XSK_UTIL_H
#define XSK_UTIL_H

#include <stdbool.h>
#include <stdint.h>

struct xsk_ctx;

struct xsk_config {
	const char *ifname;
	uint32_t queue_id;
	uint32_t frame_size;	/* 2048 for igc zero-copy */
	uint32_t num_frames;	/* 256 typical */
	bool zero_copy;
	bool need_wakeup;
};

struct xsk_ctx *xsk_ctx_create(const struct xsk_config *cfg);
void xsk_ctx_destroy(struct xsk_ctx *ctx);
int xsk_ctx_fd(const struct xsk_ctx *ctx);
uint32_t xsk_ctx_rx_peek(struct xsk_ctx *ctx, uint32_t max_batch);
const uint8_t *xsk_ctx_rx_packet(struct xsk_ctx *ctx, uint32_t idx,
				 uint32_t *len);
void xsk_ctx_rx_release(struct xsk_ctx *ctx, uint32_t count);
bool xsk_ctx_needs_wakeup(const struct xsk_ctx *ctx);

int xsk_configure_flow_steering(const char *ifname, const uint8_t *dst_mac,
				uint32_t queue_id);

#endif /* XSK_UTIL_H */
