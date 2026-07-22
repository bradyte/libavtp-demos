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

/* CRF Listener — media clock recovery
 *
 * Receives IEEE 1722 CRF timestamps over TSN and recovers the media clock
 * with a 1:1 PLL servo: one feedback edge per CRF timestamp, phase error
 * between them driving a PI loop that steers the local clock.
 *
 * Rates come from the CRF stream profile, not from constants here. A 48 kHz
 * base frequency with a timestamp interval of 160 gives 300 edges/s.
 *
 * Everything platform-specific — where edges are captured and how the clock
 * is steered — is behind media-clock.h. See mc-bcm2711.c for the CM4 with an
 * i226 and a CS2600. Select with -B, or let it autodetect.
 *
 * Build:
 *   ninja -C build examples/hw-demos/aaf-crf/crf-listener-hw
 *
 * Run:
 *   sudo chrt -f 80 taskset -c 2 ./build/examples/hw-demos/aaf-crf/crf-listener-hw \
 *       -i eth1 -d 91:E0:F0:00:FE:00
 */

#include <argp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/ptp_clock.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "crf-profile.h"
#include "crf-receiver.h"
#include "media-clock.h"
#include "pi-servo.h"

/* CRF stream profile. The PWM output rate and the servo's phase window are
 * both derived from it, so the edge rate the hardware generates always
 * matches the stream the receiver accepts. */
static struct crf_profile profile;
static unsigned int edge_freq_hz;	/* base_freq / timestamp_interval */
static int64_t edge_tolerance_ns;	/* half an edge period */

static char ifname[IFNAMSIZ];
static uint8_t crf_macaddr[ETH_ALEN];

static const struct media_clock_ops *mc_ops;
static struct media_clock *mc;
static struct pi_servo *servo;
static struct pi_servo_config servo_cfg;
static struct crf_receiver *crf_rx;

/* Wait up to two edge periods before calling an edge missed. */
static int edge_timeout_ms;

static volatile bool running = true;

/* Bootstrap state */
static bool synchronized;

/* Statistics */
static uint64_t servo_update_count;
static uint64_t pkt_dropped;
static uint64_t edge_miss_count;
static uint64_t last_edge_ts;

/* Backend device path, meaning depends on the backend */
static char device_path[64];
static char backend_name[16];

/* CSV logging */
static FILE *csv_log;
static unsigned int freeze_after;
static bool dco_frozen;
static bool verbose;

static struct argp_option options[] = {
	{"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address"},
	{"ifname", 'i', "IFNAME", 0, "Network Interface"},
	{"i2c", 'b', "PATH", 0, "Backend device path (CM4: I2C bus for CS2600)"},
	{"backend", 'B', "NAME", 0, "Media clock backend (default: autodetect)"},
	{"log", 'l', "FILE", 0, "CSV log file"},
	{"freeze", 'f', "N", 0, "Freeze DCO after N servo updates"},
	{"verbose", 'v', 0, 0, "Print per-edge EXTTS jitter to stdout"},
	{ 0 }
};

static error_t parser(int key, char *arg, struct argp_state *state)
{
	int res;

	switch (key) {
	case 'd':
		res = sscanf(arg, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
			&crf_macaddr[0], &crf_macaddr[1], &crf_macaddr[2],
			&crf_macaddr[3], &crf_macaddr[4], &crf_macaddr[5]);
		if (res != 6) {
			fprintf(stderr, "Invalid address\n");
			exit(EXIT_FAILURE);
		}
		break;
	case 'i':
		strncpy(ifname, arg, sizeof(ifname) - 1);
		break;
	case 'b':
		strncpy(device_path, arg, sizeof(device_path) - 1);
		break;
	case 'B':
		strncpy(backend_name, arg, sizeof(backend_name) - 1);
		break;
	case 'l':
		csv_log = fopen(arg, "w");
		if (!csv_log) {
			fprintf(stderr, "Failed to open log file: %s\n", arg);
			exit(EXIT_FAILURE);
		}
		break;
	case 'f':
		freeze_after = atoi(arg);
		break;
	case 'v':
		verbose = true;
		break;
	}

	return 0;
}

static struct argp argp = { options, parser };

static void sig_handler(int signum)
{
	running = false;
}

/* Diagnostics for the unsynchronised state.
 *
 * A failed synchronize() is silent by nature — there is no edge to report —
 * so "Waiting for CRF stream..." followed by nothing covers three unrelated
 * faults: no CRF arriving, CRF arriving but no feedback edges, or edges
 * present but outside tolerance. Accumulate what was actually seen and report it
 * periodically so the three are distinguishable without a packet capture. */
static uint64_t sync_ts_seen;
static uint64_t sync_edges_seen;
static int64_t sync_last_delta;

static void sync_report(void)
{
	if (!sync_edges_seen) {
		fprintf(stderr, "waiting: %" PRIu64 " CRF timestamps, no "
			"feedback edges — check the %s clock path\n",
			sync_ts_seen, mc_ops->name);
		return;
	}

	fprintf(stderr, "waiting: %" PRIu64 " CRF timestamps, %" PRIu64
		" edges, last delta=%+" PRId64 " us (tolerance ±%" PRId64 " us)\n",
		sync_ts_seen, sync_edges_seen, sync_last_delta / 1000,
		edge_tolerance_ns / 1000);
}

/* Synchronize: discard edges older than crf_ts, keep first one within tolerance */
static bool synchronize(uint64_t crf_ts)
{
	uint64_t edge_ts;

	sync_ts_seen++;

	while (mc_ops->edge_read(mc, &edge_ts, 0) == 0) {
		int64_t delta = (int64_t)(edge_ts - crf_ts);

		sync_edges_seen++;
		sync_last_delta = delta;

		if (llabs(delta) <= edge_tolerance_ns) {
			fprintf(stderr, "Synchronized: delta=%+" PRId64 "ns"
				" (after %" PRIu64 " timestamps, %" PRIu64
				" edges)\n",
				delta, sync_ts_seen, sync_edges_seen);
			sync_ts_seen = 0;
			sync_edges_seen = 0;
			return true;
		}

		if (delta < 0)
			continue;

		break;
	}

	/* One line per second of being stuck. */
	if (edge_freq_hz && sync_ts_seen % edge_freq_hz == 0)
		sync_report();

	return false;
}

static void on_crf_timestamps(uint64_t *timestamps, int count, uint8_t seq,
			      void *ctx)
{
	if (verbose) {
		uint64_t now = mc_ops->now_tai_ns(mc);
		int64_t transit = (int64_t)(now - timestamps[count - 1]);
		fprintf(stdout, "transit_us=%" PRId64 "\n", transit / 1000);
	}

	for (int i = 0; i < count; i++) {
		if (!synchronized) {
			if (!synchronize(timestamps[i]))
				continue;
			synchronized = true;
			continue;
		}

		uint64_t edge_ts;
		int res;

		res = mc_ops->edge_read(mc, &edge_ts, edge_timeout_ms);
		if (res < 0) {
			edge_miss_count++;
			continue;
		}

		/* if (verbose && last_edge_ts) {
			int64_t jitter = (int64_t)(edge_ts - last_edge_ts) - 3333333;
			fprintf(stdout, "extts_jitter_ns=%+" PRId64 "\n", jitter);
		} */
		last_edge_ts = edge_ts;

		int64_t phase_error = (int64_t)(edge_ts - timestamps[i]);

		if (llabs(phase_error) > edge_tolerance_ns) {
			edge_miss_count++;
			synchronized = false;
			continue;
		}

		enum servo_state state;
		double ppb = pi_servo_sample(servo, phase_error, edge_ts, &state);

		if (state != SERVO_UNLOCKED && !dco_frozen)
			mc_ops->adjust(mc, ppb);

		servo_update_count++;

		if (freeze_after && servo_update_count == freeze_after && !dco_frozen) {
			dco_frozen = true;
			fprintf(stderr, "*** DCO FROZEN at update %" PRIu64
				" (ppb=%.1f) ***\n",
				servo_update_count, ppb);
		}

		if (servo_update_count <= 10 || servo_update_count % 300 == 0) {
			fprintf(stderr, "upd=%6" PRIu64 "  phase=%+8" PRId64
				"ns  ppb=%+9.1f  state=%d\n",
				servo_update_count, phase_error, ppb, state);
		}

		if (csv_log) {
			fprintf(csv_log, "%" PRIu64 ",%" PRId64 ",%.3f,%.3f,%d,%u\n",
				servo_update_count, phase_error, ppb,
				pi_servo_get_drift(servo), state,
				mc_ops->actuator_code ?
					mc_ops->actuator_code(mc) : 0u);
			fflush(csv_log);
		}
	}
}

static void on_crf_drop(uint8_t expected, uint8_t actual, void *ctx)
{
	uint8_t gap = actual - expected;

	pkt_dropped += gap;

	/* Drain the stale edges that accumulated during the gap. One edge per
	 * CRF timestamp, so the count follows from the stream profile. */
	unsigned int stale = gap * profile.timestamps_per_pdu;
	for (unsigned int i = 0; i < stale; i++) {
		uint64_t discard;
		if (mc_ops->edge_read(mc, &discard, 0) < 0)
			break;
	}

	fprintf(stderr, "WARNING: Dropped %d pkt(s) (seq %d→%d)\n",
		gap, expected, actual);
}

int main(int argc, char *argv[])
{
	int res, crf_fd;
	unsigned int idle_sec = 0;

	argp_parse(&argp, argc, argv, 0, NULL, NULL);

	if (ifname[0] == '\0') {
		fprintf(stderr, "Error: -i (interface) required\n");
		return 1;
	}

	bool mac_set = false;
	for (int i = 0; i < ETH_ALEN; i++) {
		if (crf_macaddr[i] != 0) { mac_set = true; break; }
	}
	if (!mac_set) {
		fprintf(stderr, "Error: -d (destination MAC) required\n");
		return 1;
	}

	crf_profile_init(&profile);
	edge_freq_hz = (unsigned int)crf_profile_edge_hz(&profile);
	edge_tolerance_ns = crf_profile_edge_tolerance_ns(&profile);

	setlinebuf(stderr);
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	mc_ops = media_clock_select(backend_name[0] ? backend_name : NULL);
	if (!mc_ops)
		return 1;

	fprintf(stderr, "CRF Listener — Hardware Clock Recovery (%s, %u Hz)\n",
		mc_ops->name, edge_freq_hz);
	fprintf(stderr, "  Servo: %u Hz (1:1 CRF-to-edge, tol ±%" PRId64 " us)\n",
		edge_freq_hz, edge_tolerance_ns / 1000);

	struct media_clock_config mc_cfg = {
		.ifname = ifname,
		.edge_hz = crf_profile_edge_hz(&profile),
		.device = device_path[0] ? device_path : NULL,
	};

	if (mc_ops->open(&mc, &mc_cfg) < 0)
		return 1;

	/* Two edge periods: long enough that ordinary jitter is not a miss,
	 * short enough that a dead feedback path is reported rather than
	 * blocking the receive loop forever. */
	edge_timeout_ms = (int)(2000.0 / crf_profile_edge_hz(&profile)) + 1;

	pi_servo_config_init(&servo_cfg);
	servo = pi_servo_create(&servo_cfg);
	if (!servo) {
		fprintf(stderr, "Failed to create servo\n");
		res = 1;
		goto out;
	}

	if (csv_log) {
		fprintf(csv_log, "servo_update,phase_error_ns,ppb,drift,servo_state,actuator\n");
		fflush(csv_log);
	}

	struct crf_receiver_config rx_cfg = {
		.ifname = ifname,
		.macaddr = crf_macaddr,
		.on_timestamps = on_crf_timestamps,
		.on_drop = on_crf_drop,
		.callback_ctx = NULL,
		.profile = &profile,
	};

	crf_rx = crf_receiver_create(&rx_cfg);
	if (!crf_rx) {
		fprintf(stderr, "Failed to create CRF receiver\n");
		res = 1;
		goto out;
	}

	crf_fd = crf_receiver_fd(crf_rx);

	fprintf(stderr, "Waiting for CRF stream...\n");

	while (running) {
		fd_set fds;
		struct timeval tv = {.tv_sec = 1};

		FD_ZERO(&fds);
		FD_SET(crf_fd, &fds);

		if (select(crf_fd + 1, &fds, NULL, NULL, &tv) < 0) {
			if (running)
				perror("select");
			break;
		}

		if (!FD_ISSET(crf_fd, &fds)) {
			/* Nothing on the socket at all, as distinct from CRF
			 * arriving but never syncing to an edge. */
			idle_sec++;
			fprintf(stderr, "waiting: no CRF packets for %u s "
				"(stream 0x%016" PRIx64 " on %s)\n",
				idle_sec, profile.stream_id, ifname);
			continue;
		}
		idle_sec = 0;
		crf_receiver_process(crf_rx);
	}

	fprintf(stderr, "\nStopping...\n");
	fprintf(stderr, "Servo updates: %" PRIu64 " (dropped %" PRIu64
		" pkts, missed %" PRIu64 " edges)\n",
		servo_update_count, pkt_dropped, edge_miss_count);

	res = 0;
out:
	if (csv_log)
		fclose(csv_log);
	crf_receiver_destroy(crf_rx);
	pi_servo_destroy(servo);
	mc_ops->close(mc);
	return res;
}
