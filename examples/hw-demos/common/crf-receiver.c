/*
 * CRF Receiver Module
 *
 * Receives CRF packets, validates per IEEE 1722, extracts timestamps,
 * and delivers them to a consumer via callback. Tracks packet sequence
 * numbers and reports drops.
 */

#include <alloca.h>
#include <errno.h>
#include <inttypes.h>
#include <linux/if_ether.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include "avtp.h"
#include "avtp_crf.h"
#include "examples/common.h"
#include "crf-common.h"
#include "crf-receiver.h"

struct crf_receiver {
	int fd;
	struct crf_stream_config cfg;
	uint8_t seq_num;
	uint8_t expected_seq_num;
	bool first_packet;
	uint64_t pkt_count;
	uint64_t drop_count;
	crf_ts_callback_fn on_timestamps;
	crf_drop_callback_fn on_drop;
	void *callback_ctx;
};

struct crf_receiver *crf_receiver_create(const struct crf_receiver_config *cfg)
{
	struct crf_receiver *rx;
	int fd;

	if (!cfg || !cfg->ifname || !cfg->macaddr || !cfg->on_timestamps)
		return NULL;

	fd = create_listener_socket(cfg->ifname, cfg->macaddr, ETH_P_TSN);
	if (fd < 0)
		return NULL;

	rx = calloc(1, sizeof(*rx));
	if (!rx) {
		close(fd);
		return NULL;
	}

	rx->fd = fd;
	rx->first_packet = true;
	rx->on_timestamps = cfg->on_timestamps;
	rx->on_drop = cfg->on_drop;
	rx->callback_ctx = cfg->callback_ctx;

	crf_config_init(&rx->cfg);

	return rx;
}

struct crf_receiver *crf_receiver_create_from_fd(int fd,
						 crf_ts_callback_fn on_timestamps,
						 crf_drop_callback_fn on_drop,
						 void *ctx)
{
	struct crf_receiver *rx;

	if (fd < 0 || !on_timestamps)
		return NULL;

	rx = calloc(1, sizeof(*rx));
	if (!rx)
		return NULL;

	rx->fd = fd;
	rx->first_packet = true;
	rx->on_timestamps = on_timestamps;
	rx->on_drop = on_drop;
	rx->callback_ctx = ctx;

	crf_config_init(&rx->cfg);

	return rx;
}

void crf_receiver_destroy(struct crf_receiver *rx)
{
	if (!rx)
		return;

	if (rx->fd >= 0)
		close(rx->fd);
	free(rx);
}

int crf_receiver_fd(const struct crf_receiver *rx)
{
	return rx ? rx->fd : -1;
}

int crf_receiver_process(struct crf_receiver *rx)
{
	struct avtp_crf_pdu *pdu = alloca(CRF_PDU_SIZE);
	uint64_t timestamps[CRF_TIMESTAMPS_PER_PKT];
	uint8_t seq;
	int num_ts;
	ssize_t n;

	memset(pdu, 0, CRF_PDU_SIZE);
	n = recv(rx->fd, pdu, CRF_PDU_SIZE, 0);
	if (n < 0)
		return (errno == EAGAIN || errno == EINTR) ? 0 : -1;

	if (n != CRF_PDU_SIZE)
		return 0;

	if (!crf_pdu_validate(pdu, &rx->cfg, &rx->seq_num))
		return 0;

	seq = rx->seq_num - 1;

	if (!rx->first_packet) {
		uint8_t gap = seq - rx->expected_seq_num;
		if (gap != 0) {
			rx->drop_count += gap;
			if (rx->on_drop)
				rx->on_drop(rx->expected_seq_num, seq,
					    rx->callback_ctx);
		}
	}
	rx->first_packet = false;
	rx->expected_seq_num = seq + 1;

	rx->pkt_count++;

	num_ts = crf_pdu_get_timestamps(pdu, timestamps, CRF_TIMESTAMPS_PER_PKT);
	if (num_ts != CRF_TIMESTAMPS_PER_PKT)
		return 0;

	rx->on_timestamps(timestamps, num_ts, seq, rx->callback_ctx);

	return 0;
}

uint64_t crf_receiver_wait_first(struct crf_receiver *rx, int timeout_sec)
{
	struct avtp_crf_pdu *pdu = alloca(CRF_PDU_SIZE);
	uint64_t timestamps[CRF_TIMESTAMPS_PER_PKT];
	int elapsed = 0;

	while (elapsed < timeout_sec) {
		struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
		fd_set readfds;
		ssize_t n;
		int res, num_ts;

		FD_ZERO(&readfds);
		FD_SET(rx->fd, &readfds);

		res = select(rx->fd + 1, &readfds, NULL, NULL, &tv);
		if (res <= 0) {
			elapsed++;
			continue;
		}

		memset(pdu, 0, CRF_PDU_SIZE);
		n = recv(rx->fd, pdu, CRF_PDU_SIZE, 0);
		if (n != CRF_PDU_SIZE)
			continue;

		if (!crf_pdu_validate(pdu, &rx->cfg, &rx->seq_num))
			continue;

		rx->first_packet = false;
		rx->expected_seq_num = (rx->seq_num - 1) + 1;
		rx->pkt_count++;

		num_ts = crf_pdu_get_timestamps(pdu, timestamps,
						CRF_TIMESTAMPS_PER_PKT);
		if (num_ts == CRF_TIMESTAMPS_PER_PKT)
			rx->on_timestamps(timestamps, num_ts,
					  rx->seq_num - 1, rx->callback_ctx);

		return crf_pdu_get_ts0(pdu);
	}

	return 0;
}

uint64_t crf_receiver_pkt_count(const struct crf_receiver *rx)
{
	return rx ? rx->pkt_count : 0;
}

uint64_t crf_receiver_drop_count(const struct crf_receiver *rx)
{
	return rx ? rx->drop_count : 0;
}
