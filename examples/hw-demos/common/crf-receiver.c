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
#include "crf-profile.h"
#include "crf-receiver.h"

struct crf_receiver {
	int fd;
	struct crf_profile profile;
	enum crf_strictness mode;
	bool reported_mismatch;
	uint8_t expected_seq_num;
	bool first_packet;
	uint64_t pkt_count;
	uint64_t drop_count;
	crf_ts_callback_fn on_timestamps;
	crf_drop_callback_fn on_drop;
	void *callback_ctx;
};

void crf_receiver_set_strict(struct crf_receiver *rx, bool strict)
{
	if (rx)
		rx->mode = strict ? CRF_STRICT : CRF_LENIENT;
}

const struct crf_profile *crf_receiver_profile(const struct crf_receiver *rx)
{
	return rx ? &rx->profile : NULL;
}

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

	if (cfg->profile)
		rx->profile = *cfg->profile;
	else
		crf_profile_init(&rx->profile);

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

	crf_profile_init(&rx->profile);

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

/* Report a non-conformant field once per stream. Under CRF_LENIENT the PDU is
 * still delivered, so without this the mismatch would go unseen. */
static void report_mismatch(struct crf_receiver *rx,
			    const struct crf_pdu_info *info,
			    enum crf_pdu_status status)
{
	if (!info->bad_field || rx->reported_mismatch)
		return;

	rx->reported_mismatch = true;
	fprintf(stderr,
		"CRF: %s field '%s' = %" PRIu64 ", profile expects %" PRIu64
		"%s\n",
		status == CRF_PDU_OK ? "non-conformant" : "rejected",
		info->bad_field, info->bad_got, info->bad_want,
		status == CRF_PDU_OK ? " (accepted; --strict would drop it)"
				     : "");
}

int crf_receiver_process(struct crf_receiver *rx)
{
	size_t pdu_size = crf_profile_pdu_size(&rx->profile);
	struct avtp_crf_pdu *pdu = alloca(pdu_size);
	struct crf_pdu_info info;
	enum crf_pdu_status status;
	uint8_t seq;
	ssize_t n;

	memset(pdu, 0, pdu_size);
	n = recv(rx->fd, pdu, pdu_size, 0);
	if (n < 0)
		return (errno == EAGAIN || errno == EINTR) ? 0 : -1;

	status = crf_pdu_parse(pdu, (size_t)n, &rx->profile, rx->mode, &info);
	if (status != CRF_PDU_OK) {
		if (status == CRF_PDU_PROFILE)
			report_mismatch(rx, &info, status);
		return 0;
	}
	report_mismatch(rx, &info, status);

	seq = info.seq_num;

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

	rx->on_timestamps(info.timestamps, info.count, seq, rx->callback_ctx);

	return 0;
}

uint64_t crf_receiver_wait_first(struct crf_receiver *rx, int timeout_sec)
{
	size_t pdu_size = crf_profile_pdu_size(&rx->profile);
	struct avtp_crf_pdu *pdu = alloca(pdu_size);
	struct crf_pdu_info info;
	int elapsed = 0;

	while (elapsed < timeout_sec) {
		struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
		fd_set readfds;
		ssize_t n;
		int res;

		FD_ZERO(&readfds);
		FD_SET(rx->fd, &readfds);

		res = select(rx->fd + 1, &readfds, NULL, NULL, &tv);
		if (res <= 0) {
			elapsed++;
			continue;
		}

		memset(pdu, 0, pdu_size);
		n = recv(rx->fd, pdu, pdu_size, 0);
		if (n < 0)
			continue;

		if (crf_pdu_parse(pdu, (size_t)n, &rx->profile, rx->mode,
				  &info) != CRF_PDU_OK)
			continue;

		rx->first_packet = false;
		rx->expected_seq_num = info.seq_num + 1;
		rx->pkt_count++;

		rx->on_timestamps(info.timestamps, info.count, info.seq_num,
				  rx->callback_ctx);

		return info.timestamps[0];
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
