/*
 * Copyright (c) 2018, Intel Corporation
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

/* CRF Talker — Hardware Timestamped
 *
 * Captures 300Hz edges from FPGA via i226 SDP0 hardware timestamps,
 * packs into IEEE 1722 CRF packets, and transmits at 50Hz.
 *
 * No filtering, no clock logic — all intelligence lives in the listener.
 *
 * Hardware: FPGA 300Hz output → i226 SDP0 pin
 * PHC provides sub-nanosecond timestamps of each rising edge.
 *
 * Build:
 *   ninja -C build examples/hw-demos/aaf-crf/crf-talker-hw
 *
 * Run:
 *   sudo chrt -f 80 taskset -c 2 ./build/examples/hw-demos/aaf-crf/crf-talker-hw \
 *       -i eth1 -d 91:E0:F0:00:FE:00
 */

#include <alloca.h>
#include <argp.h>
#include <arpa/inet.h>
#include <inttypes.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/ptp_clock.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include "avtp.h"
#include "avtp_crf.h"
#include "examples/common.h"
#include "crf-profile.h"
#include "phc-utils.h"
#include "ptp-extts.h"

/* CRF stream profile — the same definition the listener validates against.
 * Table 28 defaults; the PDU header and the expected edge rate both come
 * from it. */
static struct crf_profile profile;

static char ifname[IFNAMSIZ];
static int verbose;
static uint8_t macaddr[ETH_ALEN];
static volatile int running = 1;
static uint64_t last_edge_ts;
static uint64_t stall_count;

#define REARM_ATTEMPTS_BEFORE_RESET	3
#define LINK_RESET_SETTLE_MS		2000

static int link_bounce(const char *iface)
{
	struct ifreq ifr;
	int sock;

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
		return -1;

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);

	/* Bring down */
	if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0)
		goto err;
	ifr.ifr_flags &= ~IFF_UP;
	if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0)
		goto err;

	usleep(100000);

	/* Bring up */
	ifr.ifr_flags |= IFF_UP;
	if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0)
		goto err;

	close(sock);
	return 0;
err:
	close(sock);
	return -1;
}

static struct argp_option options[] = {
	{"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address" },
	{"ifname", 'i', "IFNAME", 0, "Network Interface" },
	{"verbose", 'v', 0, 0, "Print per-edge EXTTS jitter to stdout" },
	{ 0 }
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
	case 'v':
		verbose = 1;
		break;
	}

	return 0;
}

static struct argp argp = { options, parser };

static void sig_handler(int signum)
{
	running = 0;
}

int main(int argc, char *argv[])
{
	int sk_fd, res;
	int ptp_fd = -1;
	clockid_t phc_clk;
	uint8_t seq_num = 0;
	int ts_idx = 0;
	uint64_t pkt_count = 0;
	struct sockaddr_ll sk_addr = {0};
	struct avtp_crf_pdu *pdu;
	size_t pdu_size;

	crf_profile_init(&profile);
	pdu_size = crf_profile_pdu_size(&profile);
	pdu = alloca(pdu_size);

	argp_parse(&argp, argc, argv, 0, NULL, NULL);

	if (ifname[0] == '\0') {
		fprintf(stderr, "Error: -i <ifname> required\n");
		return 1;
	}

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	res = phc_open(ifname, &ptp_fd, &phc_clk, NULL);
	if (res < 0) {
		fprintf(stderr, "Failed to open PHC for %s\n", ifname);
		return 1;
	}

	/* SDP0 (pin 0) → extts, channel 1. GPS PPS uses SDP2. */
	res = ptp_pin_setfunc(ptp_fd, 0, 1, 1);
	if (res < 0) {
		fprintf(stderr, "Failed to configure SDP0 pin\n");
		goto err_phc;
	}

	ptp_extts_disable(ptp_fd, 1);

	res = ptp_extts_enable(ptp_fd, 1, 1);
	if (res < 0) {
		fprintf(stderr, "Failed to enable extts on channel 1\n");
		goto err_phc;
	}

	sk_fd = create_talker_socket(-1);
	if (sk_fd < 0)
		goto err_extts;

	res = setup_socket_address(sk_fd, ifname, macaddr, ETH_P_TSN, &sk_addr);
	if (res < 0)
		goto err_sk;

	res = crf_pdu_init(pdu, &profile);
	if (res < 0)
		goto err_sk;

	fprintf(stderr, "CRF Talker — hardware timestamped (SDP0, channel 1)\n");
	fprintf(stderr, "  Interface: %s\n", ifname);
	fprintf(stderr, "  Destination: %02x:%02x:%02x:%02x:%02x:%02x\n",
		macaddr[0], macaddr[1], macaddr[2],
		macaddr[3], macaddr[4], macaddr[5]);
	fprintf(stderr, "Waiting for edges on SDP0...\n");

	while (running) {
		struct ptp_extts_event event;
		fd_set fds;
		struct timeval tv;
		ssize_t n;

		FD_ZERO(&fds);
		FD_SET(ptp_fd, &fds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;

		res = select(ptp_fd + 1, &fds, NULL, NULL, &tv);
		if (res < 0) {
			if (running)
				perror("select");
			break;
		}

		if (res == 0) {
			uint64_t now = phc_gettime_ns(phc_clk);
			uint64_t gap_ms = last_edge_ts ?
				(now - last_edge_ts) / 1000000 : 0;

			stall_count++;
			fprintf(stderr, "WARNING: extts stall #%lu"
				" — no edge for %lums (PTP %lu"
				".%03lus)\n",
				(unsigned long)stall_count,
				(unsigned long)gap_ms,
				(unsigned long)(now / NSEC_PER_SEC),
				(unsigned long)((now % NSEC_PER_SEC) / 1000000));

			if (stall_count % REARM_ATTEMPTS_BEFORE_RESET != 0) {
				/* Try ioctl re-arm first */
				fprintf(stderr, "  re-arming extts channels\n");
				ptp_extts_disable(ptp_fd, 1);
				ptp_pin_setfunc(ptp_fd, 0, 1, 1);
				ptp_extts_enable(ptp_fd, 1, 1);

				ptp_extts_disable(ptp_fd, 0);
				ptp_pin_setfunc(ptp_fd, 2, 1, 0);
				ptp_extts_enable(ptp_fd, 0, 1);
			} else {
				/* Escalate: link bounce to reset igc hardware */
				fprintf(stderr, "  re-arm failed %d times,"
					" bouncing %s link\n",
					REARM_ATTEMPTS_BEFORE_RESET, ifname);

				if (link_bounce(ifname) < 0) {
					perror("  link bounce failed");
				} else {
					fprintf(stderr, "  link reset, waiting"
						" %dms for settle\n",
						LINK_RESET_SETTLE_MS);
					usleep(LINK_RESET_SETTLE_MS * 1000);

					/* Re-configure pins after reset */
					ptp_pin_setfunc(ptp_fd, 0, 1, 1);
					ptp_extts_enable(ptp_fd, 1, 1);
					ptp_pin_setfunc(ptp_fd, 2, 1, 0);
					ptp_extts_enable(ptp_fd, 0, 1);

					fprintf(stderr, "  extts re-armed"
						" after link reset\n");
				}
			}

			ts_idx = 0;
			continue;
		}

		if (read(ptp_fd, &event, sizeof(event)) != sizeof(event))
			continue;

		if (event.index != 1)
			continue;

		uint64_t ts = (uint64_t)event.t.sec * NSEC_PER_SEC +
			      event.t.nsec;

		if (stall_count && ts_idx == 0) {
			fprintf(stderr, "  extts recovered — edge at PTP %lu"
				".%09lus\n",
				(unsigned long)(ts / NSEC_PER_SEC),
				(unsigned long)(ts % NSEC_PER_SEC));
			stall_count = 0;
		}

		/* if (verbose && last_edge_ts) {
			int64_t jitter = (int64_t)(ts - last_edge_ts) - 3333333;
			fprintf(stdout, "extts_jitter_ns=%+" PRId64 "\n", jitter);
		} */

		last_edge_ts = ts;
		pdu->crf_data[ts_idx++] = htobe64(ts);

		if (ts_idx == profile.timestamps_per_pdu) {
			res = avtp_crf_pdu_set(pdu, AVTP_CRF_FIELD_SEQ_NUM,
					       seq_num++);
			if (res < 0)
				break;

			n = sendto(sk_fd, pdu, pdu_size, 0,
				   (struct sockaddr *)&sk_addr, sizeof(sk_addr));
			if (n < 0) {
				perror("Failed to send data");
				break;
			}

			if (n != (ssize_t)pdu_size) {
				fprintf(stderr, "wrote %zd bytes, expected %zu\n",
					n, pdu_size);
			}

			pkt_count++;
			ts_idx = 0;

			if (pkt_count % 50 == 0) {
				uint64_t ts0 = be64toh(pdu->crf_data[0]);
				uint64_t ts1 = be64toh(pdu->crf_data[1]);

				fprintf(stderr, "pkt %5" PRIu64 "  ts0=%" PRIu64
					"  spacing=%ldns\n",
					pkt_count, ts0, (long)(ts1 - ts0));
			}
		}
	}

	fprintf(stderr, "\nStopping — sent %" PRIu64 " packets (%" PRIu64
		" timestamps)\n", pkt_count,
		pkt_count * profile.timestamps_per_pdu);

err_sk:
	close(sk_fd);
err_extts:
	ptp_extts_disable(ptp_fd, 1);
err_phc:
	phc_close(ptp_fd);
	return 0;
}
