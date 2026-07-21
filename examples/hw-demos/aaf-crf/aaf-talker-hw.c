/*
 * AAF Talker with ALSA capture and configurable presentation time.
 *
 * Captures audio from an ALSA device (I2S slave to CS2600) and streams
 * it as IEEE 1722 AAF packets over TSN. Presentation time is set to
 * now + configurable offset (microseconds).
 *
 * Usage:
 *   sudo ./aaf-talker-hw -i eth1 -d 91:E0:F0:00:FE:01 -t 2000 -a hw:1,0
 */

#include <alsa/asoundlib.h>
#include <argp.h>
#include <arpa/inet.h>
#include <byteswap.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "avtp.h"
#include "avtp_aaf.h"
#include "examples/common.h"

#define STREAM_ID		0xAABBCCDDEEFF0002
#define NUM_CHANNELS		2
#define SAMPLE_SIZE		4	/* S32_BE = 4 bytes */
#define FRAMES_PER_PDU		6
#define DATA_LEN		(SAMPLE_SIZE * NUM_CHANNELS * FRAMES_PER_PDU)
#define PDU_SIZE		(sizeof(struct avtp_stream_pdu) + DATA_LEN)
#define NSEC_PER_SEC		1000000000ULL
#define NSEC_PER_USEC		1000ULL

static char ifname[IFNAMSIZ];
static uint8_t macaddr[ETH_ALEN];
static int priority = -1;
static uint32_t ptime_offset_us = 2000;
static char alsa_dev[64] = "";
static int num_pkts = 0;	/* 0 = unlimited */

static struct argp_option options[] = {
	{"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address"},
	{"ifname", 'i', "IFNAME", 0, "Network Interface"},
	{"ptime", 't', "USEC", 0, "Presentation time offset in microseconds"},
	{"alsa-dev", 'a', "DEV", 0, "ALSA capture device (omit for test pattern)"},
	{"count", 'n', "NUM", 0, "Number of packets to send (0=unlimited)"},
	{"prio", 'p', "NUM", 0, "SO_PRIORITY to be set in socket"},
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
	case 'a':
		strncpy(alsa_dev, arg, sizeof(alsa_dev) - 1);
		break;
	case 'n':
		num_pkts = atoi(arg);
		break;
	case 'p':
		priority = atoi(arg);
		break;
	}

	return 0;
}

static struct argp argp = { options, parser };

static int init_pdu(struct avtp_stream_pdu *pdu)
{
	int res;

	res = avtp_aaf_pdu_init(pdu);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_TV, 1);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_STREAM_ID, STREAM_ID);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_FORMAT,
			       AVTP_AAF_FORMAT_INT_32BIT);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_NSR,
			       AVTP_AAF_PCM_NSR_48KHZ);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_CHAN_PER_FRAME,
			       NUM_CHANNELS);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_BIT_DEPTH, 32);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_STREAM_DATA_LEN, DATA_LEN);
	if (res < 0)
		return -1;

	res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_SP,
			       AVTP_AAF_PCM_SP_NORMAL);
	if (res < 0)
		return -1;

	return 0;
}

static snd_pcm_t *open_alsa_capture(void)
{
	snd_pcm_t *pcm;
	snd_pcm_hw_params_t *params;
	unsigned int rate = 48000;
	int res;

	res = snd_pcm_open(&pcm, alsa_dev, SND_PCM_STREAM_CAPTURE, 0);
	if (res < 0) {
		fprintf(stderr, "ALSA open failed: %s\n", snd_strerror(res));
		return NULL;
	}

	snd_pcm_hw_params_alloca(&params);
	snd_pcm_hw_params_any(pcm, params);

	snd_pcm_hw_params_set_access(pcm, params,
				     SND_PCM_ACCESS_RW_INTERLEAVED);
	snd_pcm_hw_params_set_format(pcm, params, SND_PCM_FORMAT_S32_LE);
	snd_pcm_hw_params_set_channels(pcm, params, NUM_CHANNELS);
	snd_pcm_hw_params_set_rate_near(pcm, params, &rate, 0);

	snd_pcm_uframes_t period = 64;
	snd_pcm_hw_params_set_period_size_near(pcm, params, &period, 0);

	snd_pcm_uframes_t buffer = period * 8;
	snd_pcm_hw_params_set_buffer_size_near(pcm, params, &buffer);

	res = snd_pcm_hw_params(pcm, params);
	if (res < 0) {
		fprintf(stderr, "ALSA hw_params failed: %s\n",
			snd_strerror(res));
		snd_pcm_close(pcm);
		return NULL;
	}

	fprintf(stderr, "ALSA capture: %s, S32_LE, %u Hz, %d ch, "
		"period=%lu, buffer=%lu\n",
		alsa_dev, rate, NUM_CHANNELS, period, buffer);

	return pcm;
}

static uint32_t calc_ptime(void)
{
	struct timespec ts;
	uint64_t now, ptime;

	clock_gettime(CLOCK_REALTIME, &ts);
	now = (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
	ptime = now + (uint64_t)ptime_offset_us * NSEC_PER_USEC;

	return ptime % (1ULL << 32);
}

static void generate_test_pattern(int32_t *buf, int pkt_num)
{
	for (int i = 0; i < FRAMES_PER_PDU; i++) {
		int32_t val = pkt_num * FRAMES_PER_PDU + i + 1;
		buf[i * NUM_CHANNELS + 0] = val;	/* L = sample counter */
		buf[i * NUM_CHANNELS + 1] = val;	/* R = same */
	}
}

int main(int argc, char *argv[])
{
	int fd, res;
	struct sockaddr_ll sk_addr;
	struct avtp_stream_pdu *pdu = alloca(PDU_SIZE);
	snd_pcm_t *pcm = NULL;
	uint8_t seq_num = 0;
	int32_t buf[FRAMES_PER_PDU * NUM_CHANNELS];
	bool use_alsa;

	argp_parse(&argp, argc, argv, 0, NULL, NULL);

	use_alsa = (alsa_dev[0] != '\0');

	if (use_alsa) {
		pcm = open_alsa_capture();
		if (!pcm)
			return 1;
	}

	fd = create_talker_socket(priority);
	if (fd < 0)
		return 1;

	res = setup_socket_address(fd, ifname, macaddr, ETH_P_TSN, &sk_addr);
	if (res < 0)
		goto err;

	res = init_pdu(pdu);
	if (res < 0)
		goto err;

	if (use_alsa)
		fprintf(stderr, "Streaming: ptime_offset=%u us, %d frames/pdu\n",
			ptime_offset_us, FRAMES_PER_PDU);
	else
		fprintf(stderr, "Test pattern: %d packets, %d frames/pdu, "
			"ptime_offset=%u us\n",
			num_pkts ? num_pkts : -1, FRAMES_PER_PDU,
			ptime_offset_us);

	for (int pkt = 0; num_pkts == 0 || pkt < num_pkts; pkt++) {
		ssize_t n;
		uint32_t avtp_time;

		if (use_alsa) {
			snd_pcm_sframes_t frames;

			frames = snd_pcm_readi(pcm, buf, FRAMES_PER_PDU);
			if (frames < 0) {
				frames = snd_pcm_recover(pcm, frames, 0);
				if (frames < 0) {
					fprintf(stderr, "ALSA read error: %s\n",
						snd_strerror(frames));
					break;
				}
				continue;
			}
			if (frames != FRAMES_PER_PDU)
				continue;
		} else {
			generate_test_pattern(buf, pkt);
		}

		/* Byteswap S32_LE → S32_BE for AAF wire format */
		{
			int32_t *dst = (int32_t *)pdu->avtp_payload;
			for (int i = 0; i < FRAMES_PER_PDU * NUM_CHANNELS; i++)
				dst[i] = htonl(buf[i]);
		}

		avtp_time = calc_ptime();

		res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_TIMESTAMP,
				       avtp_time);
		if (res < 0)
			goto err;

		res = avtp_aaf_pdu_set(pdu, AVTP_AAF_FIELD_SEQ_NUM,
				       seq_num++);
		if (res < 0)
			goto err;

		n = sendto(fd, pdu, PDU_SIZE, 0,
			   (struct sockaddr *)&sk_addr, sizeof(sk_addr));
		if (n < 0) {
			perror("sendto failed");
			goto err;
		}

		if (!use_alsa)
			usleep(125);	/* ~8000 pkt/s pacing */
	}

	if (pcm)
		snd_pcm_close(pcm);
	close(fd);
	return 0;

err:
	if (pcm)
		snd_pcm_close(pcm);
	close(fd);
	return 1;
}
