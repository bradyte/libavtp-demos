/*
 * SDP Edge Capture Diagnostic
 *
 * Starts PWM at 2kHz on GPIO12, captures hardware-timestamped edges
 * from i226 SDP0, and logs every timestamp to CSV for offline analysis.
 * Validates that i226 EXTTS can keep up with 2kHz capture rate.
 *
 * Build:
 *   gcc -O2 -o sdp-edge-capture sdp-edge-capture.c ../phc-utils.c \
 *       ../bcm2711-pwm-clock.c -I../.. -I../../include -lm
 *
 * Run:
 *   sudo taskset -c 3 ./sdp-edge-capture -i eth1 -o fpga_edges.csv -d 120
 */

#include <argp.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../phc-utils.h"
#include "../bcm2711-pwm-clock.h"

#define PWM_GPIO		12
#define CAPTURE_FREQ_HZ		2000
#define NOMINAL_INTERVAL_NS	(1000000000 / CAPTURE_FREQ_HZ)
#define DEFAULT_DURATION_SEC	120
#define MAX_EDGES		(DEFAULT_DURATION_SEC * CAPTURE_FREQ_HZ * 2)

static char ifname[16];
static char csv_path[256] = "edge_capture.csv";
static int duration_sec = DEFAULT_DURATION_SEC;

static int ptp_fd = -1;
static clockid_t phc_clk;
static struct bcm2711_pwm_clock *pwm_clock;
static volatile int running = 1;

static struct argp_option options[] = {
	{"ifname", 'i', "IFNAME", 0, "Network interface (required)"},
	{"output", 'o', "FILE", 0, "CSV output file (default: edge_capture.csv)"},
	{"duration", 'd', "SEC", 0, "Capture duration in seconds (default: 120)"},
	{0}
};

static error_t parser(int key, char *arg, struct argp_state *state)
{
	switch (key) {
	case 'i':
		strncpy(ifname, arg, sizeof(ifname) - 1);
		break;
	case 'o':
		strncpy(csv_path, arg, sizeof(csv_path) - 1);
		break;
	case 'd':
		duration_sec = atoi(arg);
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
	uint64_t *timestamps;
	uint64_t count = 0;
	uint64_t start_time, now;
	uint64_t duration_ns;
	FILE *csv;
	int res;

	argp_parse(&argp, argc, argv, 0, NULL, NULL);

	if (ifname[0] == '\0') {
		fprintf(stderr, "Error: -i <ifname> required\n");
		return 1;
	}

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	timestamps = malloc(MAX_EDGES * sizeof(uint64_t));
	if (!timestamps) {
		fprintf(stderr, "Failed to allocate edge buffer\n");
		return 1;
	}

	res = phc_open(ifname, &ptp_fd, &phc_clk, NULL);
	if (res < 0) {
		fprintf(stderr, "Failed to open PHC for %s\n", ifname);
		free(timestamps);
		return 1;
	}

	/* Start PWM at 2kHz on GPIO12 */
	res = bcm2711_pwm_clock_init(PWM_GPIO, &pwm_clock);
	if (res < 0) {
		fprintf(stderr, "Failed to init PWM on GPIO %d\n", PWM_GPIO);
		goto err;
	}

	res = bcm2711_pwm_clock_start(pwm_clock, CAPTURE_FREQ_HZ);
	if (res < 0) {
		fprintf(stderr, "Failed to start PWM at %d Hz\n", CAPTURE_FREQ_HZ);
		bcm2711_pwm_clock_cleanup(pwm_clock);
		goto err;
	}

	fprintf(stderr, "PWM: GPIO%d at %d Hz\n", PWM_GPIO, CAPTURE_FREQ_HZ);

	/* SDP0 (pin 0) → extts, channel 0 */
	res = phc_pin_setfunc(ptp_fd, 0, 1, 0);
	if (res < 0) {
		fprintf(stderr, "Failed to configure SDP0 pin\n");
		goto err;
	}

	phc_extts_disable(ptp_fd, 0);

	res = phc_extts_enable(ptp_fd, 0, 1);
	if (res < 0) {
		fprintf(stderr, "Failed to enable extts on channel 0\n");
		goto err;
	}

	fprintf(stderr, "SDP Edge Capture — %d seconds, channel 1 (SDP0)\n", duration_sec);
	fprintf(stderr, "Waiting for first edge...\n");

	/* Capture first edge to start the clock */
	res = phc_extts_read(ptp_fd, 0, &timestamps[0]);
	if (res < 0) {
		fprintf(stderr, "No edges received\n");
		goto cleanup;
	}
	count = 1;
	start_time = timestamps[0];
	duration_ns = (uint64_t)duration_sec * 1000000000ULL;

	fprintf(stderr, "Capturing (expect ~%d edges)...\n",
		duration_sec * CAPTURE_FREQ_HZ);

	while (running && count < MAX_EDGES) {
		res = phc_extts_read(ptp_fd, 0, &timestamps[count]);
		if (res < 0)
			break;

		now = timestamps[count];
		count++;

		if (now - start_time >= duration_ns)
			break;

		if (count % (CAPTURE_FREQ_HZ * 10) == 0) {
			fprintf(stderr, "  %" PRIu64 " edges (%.0fs)\n",
				count, (double)(now - start_time) / 1e9);
		}
	}

	fprintf(stderr, "Captured %" PRIu64 " edges\n\n", count);

cleanup:
	phc_extts_disable(ptp_fd, 0);
	bcm2711_pwm_clock_cleanup(pwm_clock);
	phc_close(ptp_fd);

	if (count < 2) {
		fprintf(stderr, "Not enough edges for analysis\n");
		free(timestamps);
		return 1;
	}

	/* Write CSV */
	csv = fopen(csv_path, "w");
	if (!csv) {
		fprintf(stderr, "Failed to open %s\n", csv_path);
		free(timestamps);
		return 1;
	}

	fprintf(csv, "edge,timestamp_ns,interval_ns\n");
	fprintf(csv, "1,%" PRIu64 ",0\n", timestamps[0]);
	for (uint64_t i = 1; i < count; i++) {
		int64_t interval = (int64_t)(timestamps[i] - timestamps[i - 1]);
		fprintf(csv, "%" PRIu64 ",%" PRIu64 ",%" PRId64 "\n",
			i + 1, timestamps[i], interval);
	}
	fclose(csv);
	fprintf(stderr, "Written to %s\n\n", csv_path);

	/* Statistics */
	double sum = 0, sum_sq = 0;
	int64_t min_interval = INT64_MAX, max_interval = INT64_MIN;
	uint64_t within_1us = 0;

	for (uint64_t i = 1; i < count; i++) {
		int64_t interval = (int64_t)(timestamps[i] - timestamps[i - 1]);
		double diff = (double)interval;

		sum += diff;
		sum_sq += diff * diff;

		if (interval < min_interval)
			min_interval = interval;
		if (interval > max_interval)
			max_interval = interval;
		if (llabs(interval - NOMINAL_INTERVAL_NS) <= 1000)
			within_1us++;
	}

	uint64_t n = count - 1;
	double mean = sum / n;
	double variance = (sum_sq / n) - (mean * mean);
	double stddev = sqrt(variance);

	fprintf(stderr, "=== Edge Capture Statistics ===\n");
	fprintf(stderr, "  Edges:    %" PRIu64 "\n", count);
	fprintf(stderr, "  Duration: %.3f s\n", (double)(timestamps[count-1] - timestamps[0]) / 1e9);
	fprintf(stderr, "  Mean:     %.1f ns (nominal %d)\n", mean, NOMINAL_INTERVAL_NS);
	fprintf(stderr, "  Min:      %" PRId64 " ns\n", min_interval);
	fprintf(stderr, "  Max:      %" PRId64 " ns\n", max_interval);
	fprintf(stderr, "  Stddev:   %.1f ns\n", stddev);
	fprintf(stderr, "  Within ±1µs: %" PRIu64 "/%" PRIu64 " (%.2f%%)\n",
		within_1us, n, 100.0 * within_1us / n);

	free(timestamps);
	return 0;

err:
	phc_close(ptp_fd);
	free(timestamps);
	return 1;
}
