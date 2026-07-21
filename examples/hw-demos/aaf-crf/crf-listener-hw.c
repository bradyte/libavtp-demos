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

/* CRF Listener — Hardware Clock Recovery
 *
 * Receives IEEE 1722 CRF timestamps over TSN and recovers the 48kHz media
 * clock using a 1:1 PLL servo at 300Hz.
 *
 * Signal path:
 *   BCM2711 PWM (300Hz, GPIO12) → CS2600 CLK_IN
 *   CS2600 AUX_OUT (buffered 300Hz) → i226 SDP0 (feedback)
 *   CS2600 CLK_OUT (24.576MHz) → I2S MCLK
 *
 * Each CRF timestamp has exactly one feedback edge from the CS2600.
 * The PI servo adjusts the PWM fractional divider to lock phase.
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
#include "pi-hw.h"
#include "clock-adjust.h"
#include "bcm2711-pwm-clock.h"
#include "cs2600.h"
#include "phc-utils.h"

#define SERVO_MAX_PPB		2000000.0
#define SERVO_STEP_THRESH	100000000.0

#define PWM_GPIO		12

/* CRF stream profile. The PWM output rate and the servo's phase window are
 * both derived from it, so the edge rate the hardware generates always
 * matches the stream the receiver accepts. */
static struct crf_profile profile;
static unsigned int edge_freq_hz;	/* base_freq / timestamp_interval */
static int64_t edge_tolerance_ns;	/* half an edge period */

static char ifname[IFNAMSIZ];
static uint8_t crf_macaddr[ETH_ALEN];

static struct bcm2711_pwm_clock *pwm_clock;
static struct cs2600 cs2600_dev;
static struct pi_servo *servo;
static struct crf_receiver *crf_rx;
static int ptp_fd = -1;
static clockid_t phc_clk;

static clock_adjust_fn clock_adjust;
static void *clock_adjust_ctx;

static volatile bool running = true;

/* Bootstrap state */
static bool synchronized;

/* Statistics */
static uint64_t servo_update_count;
static uint64_t pkt_dropped;
static uint64_t edge_miss_count;
static uint64_t last_edge_ts;

/* I2C device for CS2600 */
static char i2c_dev[64] = "/dev/i2c-1";

/* CSV logging */
static FILE *csv_log;
static unsigned int freeze_after;
static bool dco_frozen;
static bool verbose;

static struct argp_option options[] = {
	{"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address"},
	{"ifname", 'i', "IFNAME", 0, "Network Interface"},
	{"i2c", 'b', "PATH", 0, "I2C device for CS2600 (default /dev/i2c-1)"},
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
		strncpy(i2c_dev, arg, sizeof(i2c_dev) - 1);
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

/* Non-blocking read of one extts event. Returns 0 on success, -1 if nothing ready. */
static int extts_read_nonblock(uint64_t *ts)
{
	fd_set fds;
	struct timeval tv = {0};
	struct ptp_extts_event event;

	FD_ZERO(&fds);
	FD_SET(ptp_fd, &fds);
	if (select(ptp_fd + 1, &fds, NULL, NULL, &tv) <= 0)
		return -1;
	if (read(ptp_fd, &event, sizeof(event)) != sizeof(event))
		return -1;
	if (event.index != 0)
		return -1;

	*ts = (uint64_t)event.t.sec * NSEC_PER_SEC + event.t.nsec;
	return 0;
}

/* Synchronize: discard edges older than crf_ts, keep first one within tolerance */
static bool synchronize(uint64_t crf_ts)
{
	uint64_t edge_ts;

	while (extts_read_nonblock(&edge_ts) == 0) {
		int64_t delta = (int64_t)(edge_ts - crf_ts);

		if (llabs(delta) <= edge_tolerance_ns) {
			fprintf(stderr, "Synchronized: delta=%+" PRId64 "ns\n",
				delta);
			return true;
		}

		if (delta < 0)
			continue;

		break;
	}

	return false;
}

static void on_crf_timestamps(uint64_t *timestamps, int count, uint8_t seq,
			      void *ctx)
{
	if (verbose) {
		uint64_t now = phc_gettime_ns(phc_clk);
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

		res = phc_extts_read(ptp_fd, 0, &edge_ts);
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
			clock_adjust(clock_adjust_ctx, ppb);

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
				bcm2711_pwm_clock_get_divf(pwm_clock));
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
		if (phc_extts_read(ptp_fd, 0, &discard) < 0)
			break;
	}

	fprintf(stderr, "WARNING: Dropped %d pkt(s) (seq %d→%d)\n",
		gap, expected, actual);
}

static int pwm_clock_adjust(void *ctx, double ppb)
{
	return bcm2711_pwm_clock_adjust_freq((struct bcm2711_pwm_clock *)ctx, ppb);
}

int main(int argc, char *argv[])
{
	int res, crf_fd;

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

	res = phc_open(ifname, &ptp_fd, &phc_clk, NULL);
	if (res < 0) {
		fprintf(stderr, "Failed to open PHC for %s\n", ifname);
		return 1;
	}

	fprintf(stderr, "CRF Listener — Hardware Clock Recovery (1:1 PLL, %u Hz)\n",
		edge_freq_hz);
	fprintf(stderr, "  GPIO %d: %u Hz → CS2600 CLK_IN → AUX_OUT → i226 SDP0\n",
		PWM_GPIO, edge_freq_hz);
	fprintf(stderr, "  Servo: %u Hz (1:1 CRF-to-edge, tol ±%" PRId64 " us)\n\n",
		edge_freq_hz, edge_tolerance_ns / 1000);

	res = bcm2711_pwm_clock_init(PWM_GPIO, &pwm_clock);
	if (res < 0) {
		fprintf(stderr, "Failed to init PWM\n");
		goto err_phc;
	}

	clock_adjust = pwm_clock_adjust;
	clock_adjust_ctx = pwm_clock;

	res = bcm2711_pwm_clock_prepare(pwm_clock, edge_freq_hz);
	if (res < 0) {
		fprintf(stderr, "Failed to prepare PWM\n");
		goto err_pwm;
	}

	{
		uint64_t now = phc_gettime_ns(phc_clk);
		uint64_t target = ((now / NSEC_PER_SEC) + 1) * NSEC_PER_SEC;

		fprintf(stderr, "PWM: Waiting for PTP second %lu to enable...\n",
			(unsigned long)(target / NSEC_PER_SEC));

		uint64_t wait_ns = (target - 1000000) - now;
		struct timespec dur = {
			.tv_sec = wait_ns / NSEC_PER_SEC,
			.tv_nsec = wait_ns % NSEC_PER_SEC,
		};
		clock_nanosleep(CLOCK_MONOTONIC, 0, &dur, NULL);
		while (phc_gettime_ns(phc_clk) < target)
			;

		bcm2711_pwm_clock_enable(pwm_clock);

		uint64_t actual = phc_gettime_ns(phc_clk);
		fprintf(stderr, "PWM: Enabled at PTP +%ldns from target\n",
			(long)(actual - target));
	}

	/* Initialize CS2600 jitter cleaner: 300Hz → 24.576MHz */
	res = cs2600_open(i2c_dev, CS2600_I2C_ADDR, &cs2600_dev);
	if (res < 0) {
		fprintf(stderr, "Failed to open CS2600 on %s\n", i2c_dev);
		goto err_pwm;
	}

	res = cs2600_check_id(&cs2600_dev);
	if (res < 0) {
		fprintf(stderr, "CS2600 not found (bad device ID)\n");
		goto err_cs2600;
	}

	fprintf(stderr, "CS2600: Configuring 300Hz → 24.576MHz...\n");
	res = cs2600_init_mult_300hz(&cs2600_dev);
	if (res == -2) {
		fprintf(stderr, "CS2600: Frequency lock timeout\n");
		goto err_cs2600;
	} else if (res < 0) {
		fprintf(stderr, "CS2600: Init failed\n");
		goto err_cs2600;
	}
	fprintf(stderr, "CS2600: Locked\n");

	/* Configure SDP0 for edge capture (CS2600 AUX_OUT → SDP0) */
	phc_extts_disable(ptp_fd, 0);
	res = phc_pin_setfunc(ptp_fd, 0, 1, 0);
	if (res < 0) {
		fprintf(stderr, "Failed to configure SDP0\n");
		goto err_cs2600;
	}
	res = phc_extts_enable(ptp_fd, 0, 1);
	if (res < 0) {
		fprintf(stderr, "Failed to enable SDP0 capture\n");
		goto err_cs2600;
	}

	servo = pi_servo_create(SERVO_MAX_PPB, SERVO_STEP_THRESH);
	if (!servo) {
		fprintf(stderr, "Failed to create servo\n");
		goto err_extts;
	}

	if (csv_log) {
		fprintf(csv_log, "servo_update,phase_error_ns,ppb,drift,servo_state,divf\n");
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
		goto err_servo;
	}

	crf_fd = crf_receiver_fd(crf_rx);

	fprintf(stderr, "Waiting for CRF stream...\n");

	while (running) {
		fd_set fds;
		struct timeval tv = {.tv_sec = 1};

		FD_ZERO(&fds);
		FD_SET(crf_fd, &fds);

		res = select(crf_fd + 1, &fds, NULL, NULL, &tv);
		if (res < 0) {
			if (running) perror("select");
			break;
		}

		if (FD_ISSET(crf_fd, &fds))
			crf_receiver_process(crf_rx);
	}

	fprintf(stderr, "\nStopping...\n");
	fprintf(stderr, "Servo updates: %" PRIu64 " (dropped %" PRIu64
		" pkts, missed %" PRIu64 " edges)\n",
		servo_update_count, pkt_dropped, edge_miss_count);

	if (csv_log)
		fclose(csv_log);

	phc_extts_disable(ptp_fd, 0);
	crf_receiver_destroy(crf_rx);
err_servo:
	pi_servo_destroy(servo);
err_extts:
	phc_extts_disable(ptp_fd, 0);
err_cs2600:
	cs2600_close(&cs2600_dev);
err_pwm:
	bcm2711_pwm_clock_cleanup(pwm_clock);
err_phc:
	phc_close(ptp_fd);
	return 0;
}
