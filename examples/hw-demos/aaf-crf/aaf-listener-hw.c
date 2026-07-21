/*
 * AAF Listener with ALSA playback and presentation-time gating.
 *
 * Receives IEEE 1722 AAF packets and holds each one until its presentation
 * time arrives, then writes to ALSA. Listeners at different points on the
 * network therefore present the same sample at the same instant.
 *
 * Pattern: recv → gate on ptime → present when now >= ptime
 * The 8kHz packet arrival cadence drives presentation timing.
 *
 * Presentation time is compared against CLOCK_REALTIME, which is expected to
 * be PTP-synchronised via phc2sys.
 *
 * Platform-neutral: no custom hardware, any PTP-capable NIC. Receive uses
 * AF_PACKET by default; -q selects AF_XDP on a given NIC queue where the
 * kernel supports it. A stock Raspberry Pi 5 image is built without
 * CONFIG_XDP_SOCKETS, so omit -q there and it uses AF_PACKET.
 *
 * Usage:
 *   CM4 + i226, AF_XDP on queue 0:
 *     sudo chrt -f 80 taskset -c 3 ./aaf-listener-hw \
 *         -i eth1 -d 91:E0:F0:00:FE:01 -t 500 -a hw:1,0 -q 0
 *
 *   Stock Raspberry Pi 5, AF_PACKET:
 *     sudo chrt -f 80 taskset -c 3 ./aaf-listener-hw \
 *         -i eth0 -d 91:E0:F0:00:FE:01 -t 500 -a hw:0,0
 */

#include <alsa/asoundlib.h>
#include <argp.h>
#include <errno.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>

#include "avtp.h"
#include "avtp_aaf.h"
#include "examples/common.h"
#include "aaf-profile.h"

#include "xsk-util.h"

/* AAF stream profile — the same definition the talker encodes. ALSA setup
 * and the PDU layout both derive from it. */
static struct aaf_profile profile;
static enum aaf_strictness aaf_mode = AAF_LENIENT;
static bool reported_mismatch;
#define NSEC_PER_SEC		1000000000ULL
#define NSEC_PER_USEC		1000ULL
#define BUFFER_FRAMES		4096

static char ifname[IFNAMSIZ];
static uint8_t macaddr[ETH_ALEN];
static uint32_t ptime_offset_us = 500;
static uint32_t ptime_tolerance_us = 250;
static char alsa_dev[64] = "";
static char log_path[256] = "";
static int num_pkts = 0;
static int xdp_queue = -1;
static bool xdp_required = false;
static bool verbose = false;
static uint8_t expected_seq;
static bool seq_valid;
static FILE *logfp;
static volatile sig_atomic_t running = 1;

static void sig_handler(int sig)
{
	(void)sig;
	running = 0;
}

static struct argp_option options[] = {
	{"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address"},
	{"ifname", 'i', "IFNAME", 0, "Network Interface"},
	{"ptime", 't', "USEC", 0, "Max transit time / ptime offset (default 500)"},
	{"tolerance", 'T', "USEC", 0, "Late packet tolerance (default 250)"},
	{"alsa-dev", 'a', "DEV", 0, "ALSA playback device (omit to print values)"},
	{"count", 'n', "NUM", 0, "Number of packets to receive (0=unlimited)"},
	{"log", 'l', "FILE", 0, "Log file for diagnostics"},
	{"xdp-queue", 'q', "QUEUE", 0, "NIC queue for AF_XDP receive"},
	{"xdp-required", 'X', NULL, 0, "Fail if AF_XDP unavailable"},
	{"verbose", 'v', NULL, 0, "Print per-packet transit time to stdout"},
	{0}
};

static error_t parser(int key, char *arg, struct argp_state *state)
{
	int res;

	switch (key) {
	case 'd':
		res = sscanf(arg, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
			     &macaddr[0], &macaddr[1], &macaddr[2],
			     &macaddr[3], &macaddr[4], &macaddr[5]);
		if (res != 6) {
			fprintf(stderr, "Invalid address\n");
			exit(EXIT_FAILURE);
		}
		break;
	case 'i':
		strncpy(ifname, arg, sizeof(ifname) - 1);
		break;
	case 't':
		ptime_offset_us = atoi(arg);
		break;
	case 'T':
		ptime_tolerance_us = atoi(arg);
		break;
	case 'a':
		strncpy(alsa_dev, arg, sizeof(alsa_dev) - 1);
		break;
	case 'n':
		num_pkts = atoi(arg);
		break;
	case 'l':
		strncpy(log_path, arg, sizeof(log_path) - 1);
		break;
	case 'q':
		xdp_queue = atoi(arg);
		break;
	case 'X':
		xdp_required = true;
		break;
	case 'v':
		verbose = true;
		break;
	}

	return 0;
}

static struct argp argp = { options, parser };

static snd_pcm_t *open_alsa_playback(void)
{
	snd_pcm_t *pcm;
	snd_pcm_hw_params_t *params;
	snd_pcm_sw_params_t *swparams;
	unsigned int rate = 48000;
	snd_pcm_uframes_t buffer = BUFFER_FRAMES;
	snd_pcm_uframes_t period = 1024;
	snd_pcm_uframes_t start_thr;
	int res;

	res = snd_pcm_open(&pcm, alsa_dev, SND_PCM_STREAM_PLAYBACK, 0);
	if (res < 0) {
		fprintf(stderr, "ALSA open failed: %s\n", snd_strerror(res));
		return NULL;
	}

	snd_pcm_hw_params_alloca(&params);
	snd_pcm_hw_params_any(pcm, params);

	snd_pcm_hw_params_set_access(pcm, params,
				     SND_PCM_ACCESS_RW_INTERLEAVED);
	snd_pcm_hw_params_set_format(pcm, params, SND_PCM_FORMAT_S32_LE);
	snd_pcm_hw_params_set_channels(pcm, params, profile.channels);
	snd_pcm_hw_params_set_rate_near(pcm, params, &rate, 0);
	snd_pcm_hw_params_set_period_size_near(pcm, params, &period, 0);
	snd_pcm_hw_params_set_buffer_size_near(pcm, params, &buffer);

	res = snd_pcm_hw_params(pcm, params);
	if (res < 0) {
		fprintf(stderr, "ALSA hw_params failed: %s\n",
			snd_strerror(res));
		snd_pcm_close(pcm);
		return NULL;
	}

	start_thr = (uint64_t)ptime_offset_us * 48 / 1000;
	if (start_thr < period)
		start_thr = period;

	snd_pcm_sw_params_alloca(&swparams);
	snd_pcm_sw_params_current(pcm, swparams);
	snd_pcm_sw_params_set_start_threshold(pcm, swparams, start_thr);
	snd_pcm_sw_params_set_avail_min(pcm, swparams, profile.frames_per_pdu);

	res = snd_pcm_sw_params(pcm, swparams);
	if (res < 0) {
		fprintf(stderr, "ALSA sw_params failed: %s\n",
			snd_strerror(res));
		snd_pcm_close(pcm);
		return NULL;
	}

	fprintf(stderr, "ALSA playback: %s, S32_LE, %u Hz, %d ch, "
		"period=%lu, buffer=%lu, start_thr=%lu\n",
		alsa_dev, rate, profile.channels, period, buffer, start_thr);

	return pcm;
}

/* Report a non-conformant field once per stream. Under AAF_LENIENT the PDU is
 * still played, so without this the mismatch would go unseen. */
static void report_mismatch(const struct aaf_pdu_info *info,
			    enum aaf_pdu_status status)
{
	if (!info->bad_field || reported_mismatch)
		return;

	reported_mismatch = true;
	fprintf(stderr,
		"AAF: %s field '%s' = %" PRIu64 ", profile expects %" PRIu64
		"%s\n",
		status == AAF_PDU_OK ? "non-conformant" : "rejected",
		info->bad_field, info->bad_got, info->bad_want,
		status == AAF_PDU_OK ? " (accepted; strict mode would drop it)"
				     : "");
}
static int prefill_silence(snd_pcm_t *pcm, snd_pcm_uframes_t frames)
{
	int32_t silence[1024 * 8];	/* 1024 frames, up to 8 channels */
	snd_pcm_uframes_t chunk_max = sizeof(silence) / sizeof(silence[0]) /
				      profile.channels;

	memset(silence, 0, sizeof(silence));

	while (frames > 0) {
		snd_pcm_uframes_t chunk = frames;
		if (chunk > chunk_max)
			chunk = chunk_max;

		snd_pcm_sframes_t written = snd_pcm_writei(pcm, silence, chunk);
		if (written < 0)
			return written;
		frames -= written;
	}

	return 0;
}

static void process_pdu(struct avtp_stream_pdu *pdu, size_t len,
			snd_pcm_t *pcm,
			bool use_alsa, bool *started,
			uint64_t *pkt_count, uint64_t *drop_count,
			uint64_t *late_count, uint64_t *last_log_count)
{
	struct aaf_pdu_info info;
	enum aaf_pdu_status status;
	struct timespec tspec;
	uint8_t seq;
	int res;

	status = aaf_pdu_parse(pdu, len, &profile, aaf_mode, &info);
	if (status != AAF_PDU_OK) {
		(*drop_count)++;
		report_mismatch(&info, status);
		return;
	}
	report_mismatch(&info, status);

	seq = info.seq_num;

	if (seq_valid && seq != expected_seq) {
		uint8_t gap = (uint8_t)(seq - expected_seq);
		if (logfp)
			fprintf(logfp, "Seq gap: expected %u got %u "
				"(%u frames)\n", expected_seq, seq,
				(unsigned)(gap * profile.frames_per_pdu));
	}
	expected_seq = seq + 1;
	seq_valid = true;

	res = get_presentation_time(info.timestamp, &tspec);
	if (res < 0)
		return;

	uint64_t ptime_ns = (uint64_t)tspec.tv_sec * NSEC_PER_SEC
			    + tspec.tv_nsec;
	struct timespec now_ts;
	clock_gettime(CLOCK_REALTIME, &now_ts);
	uint64_t now_ns = (uint64_t)now_ts.tv_sec * NSEC_PER_SEC
			  + now_ts.tv_nsec;

	/* Correct 32-bit wrap reconstruction */
	int64_t delta = (int64_t)(ptime_ns - now_ns);
	if (delta > (int64_t)(1ULL << 31))
		ptime_ns -= (1ULL << 32);
	else if (delta < -(int64_t)(1ULL << 31))
		ptime_ns += (1ULL << 32);
	delta = (int64_t)(ptime_ns - now_ns);

	if (verbose)
		fprintf(stdout, "transit_us=%" PRId64 " slack_us=%" PRId64 "\n",
			(int64_t)ptime_offset_us - delta / 1000, delta / 1000);

	if (!use_alsa) {
		printf("pkt %3" PRIu64 "  delta=%+" PRId64 "ns\n",
		       *pkt_count, delta);
		(*pkt_count)++;
		return;
	}

	/* Ptime gate: drop if too far past presentation time */
	int64_t tolerance_ns = (int64_t)ptime_tolerance_us * NSEC_PER_USEC;
	if (delta < -tolerance_ns) {
		(*late_count)++;
		if (logfp)
			fprintf(logfp, "Late #%" PRIu64
				": ptime %+" PRId64 "ns\n",
				*late_count, delta);
		return;
	}

	/* Prefill ALSA on first valid packet */
	if (!*started) {
		snd_pcm_uframes_t pf = BUFFER_FRAMES - profile.frames_per_pdu;
		prefill_silence(pcm, pf);
		snd_pcm_nonblock(pcm, 1);
		*started = true;
		fprintf(stderr, "Stream started: prefill=%lu frames "
			"(%.1f ms)\n", pf, pf / 48.0);
	}

	/* Depacketize and write directly to ALSA */
	size_t samples = info.payload_len / sizeof(int32_t);
	int32_t pcm_buf[samples];
	const int32_t *src = info.payload;
	for (size_t i = 0; i < samples; i++)
		pcm_buf[i] = ntohl(src[i]);

	snd_pcm_sframes_t written;
	written = snd_pcm_writei(pcm, pcm_buf, profile.frames_per_pdu);
	if (written < 0) {
		if (written != -EAGAIN) {
			fprintf(stderr, "ALSA underrun, recovering\n");
			snd_pcm_recover(pcm, written, 0);
			*started = false;
		}
		return;
	}
	(*pkt_count)++;

	if (logfp && (*pkt_count % 8000 == 0) &&
	    *pkt_count != *last_log_count) {
		*last_log_count = *pkt_count;
		fprintf(logfp, "%" PRIu64 " pkts, %" PRIu64
			" late, delta=%+" PRId64 "ns\n",
			*pkt_count, *late_count, delta);
	}
}

int main(int argc, char *argv[])
{
	int sk_fd = -1, res;
	snd_pcm_t *pcm = NULL;
	struct avtp_stream_pdu *pdu;
	size_t pdu_size;
	bool started = false;
	uint64_t pkt_count = 0;
	uint64_t drop_count = 0;
	uint64_t late_count = 0;
	uint64_t last_log_count = 0;
	bool use_alsa;
	bool use_xdp = false;
	struct xsk_ctx *xsk = NULL;

	aaf_profile_init(&profile);
	pdu_size = aaf_profile_pdu_size(&profile);
	pdu = alloca(pdu_size);

	argp_parse(&argp, argc, argv, 0, NULL, NULL);

	if (log_path[0]) {
		logfp = fopen(log_path, "w");
		if (!logfp) {
			perror("Failed to open log file");
			return 1;
		}
		setlinebuf(logfp);
	}

	use_alsa = (alsa_dev[0] != '\0');

	if (use_alsa) {
		pcm = open_alsa_playback();
		if (!pcm)
			return 1;
	}

	if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
		perror("mlockall");
		return 1;
	}

	/* Try AF_XDP if queue specified */
	if (xdp_queue >= 0) {
		if (xsk_configure_flow_steering(ifname, macaddr,
						(uint32_t)xdp_queue) < 0)
			fprintf(stderr, "Flow steering setup failed "
				"(continuing anyway)\n");

		struct xsk_config cfg = {
			.ifname = ifname,
			.queue_id = (uint32_t)xdp_queue,
			.frame_size = 2048,
			.num_frames = 256,
			.zero_copy = true,
			.need_wakeup = true,
		};
		xsk = xsk_ctx_create(&cfg);
		if (xsk) {
			use_xdp = true;
		} else if (xdp_required) {
			fprintf(stderr, "AF_XDP required but unavailable\n");
			return 1;
		} else {
			fprintf(stderr, "AF_XDP unavailable, "
				"falling back to AF_PACKET\n");
		}
	}

	if (!use_xdp) {
		sk_fd = create_listener_socket(ifname, macaddr, ETH_P_TSN);
		if (sk_fd < 0)
			return 1;
	}

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	struct pollfd fds[1];
	fds[0].fd = use_xdp ? xsk_ctx_fd(xsk) : sk_fd;
	fds[0].events = POLLIN;

	if (use_alsa)
		fprintf(stderr, "Waiting for AAF stream (ptime_offset=%u us, "
			"tolerance=%u us, %s)...\n", ptime_offset_us,
			ptime_tolerance_us,
			use_xdp ? "AF_XDP" : "AF_PACKET");
	else
		fprintf(stderr, "Print mode: waiting for %d packets...\n",
			num_pkts ? num_pkts : -1);

	while (running && (num_pkts == 0 || pkt_count < (uint64_t)num_pkts)) {
		res = poll(fds, 1, use_alsa ? -1 : 5000);
		if (res < 0) {
			if (errno == EINTR)
				break;
			perror("poll");
			goto err;
		}
		if (res == 0) {
			fprintf(stderr, "Timeout waiting for packets\n");
			break;
		}
		if (!(fds[0].revents & POLLIN))
			continue;

		if (use_xdp) {
			uint32_t rcvd = xsk_ctx_rx_peek(xsk, 32);
			for (uint32_t i = 0; i < rcvd; i++) {
				uint32_t len;
				const uint8_t *frame =
					xsk_ctx_rx_packet(xsk, i, &len);

				if (len < sizeof(struct ethhdr) + pdu_size)
					continue;

				struct ethhdr *eth = (struct ethhdr *)frame;
				if (ntohs(eth->h_proto) != ETH_P_TSN)
					continue;

				struct avtp_stream_pdu *rpdu =
					(struct avtp_stream_pdu *)
					(frame + sizeof(struct ethhdr));

				process_pdu(rpdu, pdu_size, pcm,
					    use_alsa, &started,
					    &pkt_count, &drop_count,
					    &late_count, &last_log_count);
			}
			xsk_ctx_rx_release(xsk, rcvd);
			continue;
		}
		/* AF_PACKET path */
		ssize_t n = recv(sk_fd, pdu, pdu_size, 0);
		if (n < 0 || (size_t)n != pdu_size)
			continue;

		process_pdu(pdu, pdu_size, pcm, use_alsa, &started,
			    &pkt_count, &drop_count,
			    &late_count, &last_log_count);
	}

	fprintf(stderr, "\nStopping: %" PRIu64 " pkts, %" PRIu64
		" dropped, %" PRIu64 " late\n",
		pkt_count, drop_count, late_count);

	if (logfp)
		fclose(logfp);
	if (pcm)
		snd_pcm_close(pcm);
	if (xsk)
		xsk_ctx_destroy(xsk);
	if (sk_fd >= 0)
		close(sk_fd);
	return 0;

err:
	if (logfp)
		fclose(logfp);
	if (pcm)
		snd_pcm_close(pcm);
	if (xsk)
		xsk_ctx_destroy(xsk);
	if (sk_fd >= 0)
		close(sk_fd);
	return 1;
}
