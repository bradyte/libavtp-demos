/*
 * AAF Talker with ALSA capture and configurable presentation time.
 *
 * Captures audio from an ALSA device (I2S slave to CS2600) and streams
 * it as IEEE 1722 AAF packets over TSN. Presentation time is set to
 * now + configurable offset (microseconds).
 *
 * Usage:
 *   sudo ./aaf-talker-rpi -i eth1 -d 91:E0:F0:00:FE:01 -t 2000 -a hw:1,0
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
#include "aaf-profile.h"
#include "alsa-util.h"

/* AAF stream profile — the same definition the listener validates against.
 * Channel count, sample size and PDU layout all derive from it. */
static struct aaf_profile profile;

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

static uint32_t calc_ptime(void)
{
	struct timespec ts;
	uint64_t now, ptime;

	clock_gettime(CLOCK_REALTIME, &ts);
	now = (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
	ptime = now + (uint64_t)ptime_offset_us * NSEC_PER_USEC;

	return ptime % (1ULL << 32);
}

/* Ramp: every channel of a frame carries the same running sample counter,
 * so a listener can spot dropped or reordered frames by inspection. */
static void generate_test_pattern(int32_t *buf, int pkt_num,
				  unsigned int frames, unsigned int channels)
{
	for (unsigned int i = 0; i < frames; i++) {
		int32_t val = pkt_num * frames + i + 1;

		for (unsigned int c = 0; c < channels; c++)
			buf[i * channels + c] = val;
	}
}

int main(int argc, char *argv[])
{
	int fd, res;
	struct sockaddr_ll sk_addr;
	struct avtp_stream_pdu *pdu;
	snd_pcm_t *pcm = NULL;
	uint8_t seq_num = 0;
	int32_t *buf;
	size_t pdu_size, samples;
	unsigned int frames_per_pdu;
	bool use_alsa;

	aaf_profile_init(&profile);
	pdu_size = aaf_profile_pdu_size(&profile);
	samples = aaf_profile_samples_per_pdu(&profile);
	frames_per_pdu = profile.frames_per_pdu;
	pdu = alloca(pdu_size);
	buf = alloca(samples * sizeof(*buf));

	argp_parse(&argp, argc, argv, 0, NULL, NULL);

	use_alsa = (alsa_dev[0] != '\0');

	if (use_alsa) {
		struct alsa_config acfg = {
			.device = alsa_dev,
			.stream = SND_PCM_STREAM_CAPTURE,
			.period = 64,
			.buffer = 64 * 8,
		};

		pcm = alsa_open(&acfg, &profile);
		if (!pcm)
			return 1;
	}

	fd = create_talker_socket(priority);
	if (fd < 0)
		return 1;

	res = setup_socket_address(fd, ifname, macaddr, ETH_P_TSN, &sk_addr);
	if (res < 0)
		goto err;

	res = aaf_pdu_init(pdu, &profile);
	if (res < 0)
		goto err;

	if (use_alsa)
		fprintf(stderr, "Streaming: ptime_offset=%u us, %d frames/pdu\n",
			ptime_offset_us, frames_per_pdu);
	else
		fprintf(stderr, "Test pattern: %d packets, %d frames/pdu, "
			"ptime_offset=%u us\n",
			num_pkts ? num_pkts : -1, frames_per_pdu,
			ptime_offset_us);

	for (int pkt = 0; num_pkts == 0 || pkt < num_pkts; pkt++) {
		ssize_t n;
		uint32_t avtp_time;

		if (use_alsa) {
			snd_pcm_sframes_t frames;

			frames = snd_pcm_readi(pcm, buf, frames_per_pdu);
			if (frames < 0) {
				frames = snd_pcm_recover(pcm, frames, 0);
				if (frames < 0) {
					fprintf(stderr, "ALSA read error: %s\n",
						snd_strerror(frames));
					break;
				}
				continue;
			}
			if (frames != (snd_pcm_sframes_t)frames_per_pdu)
				continue;
		} else {
			generate_test_pattern(buf, pkt, frames_per_pdu,
					      profile.channels);
		}

		/* Byteswap S32_LE → S32_BE for AAF wire format */
		{
			int32_t *dst = (int32_t *)pdu->avtp_payload;
			for (size_t i = 0; i < samples; i++)
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

		n = sendto(fd, pdu, pdu_size, 0,
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
