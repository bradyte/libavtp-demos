/*
 * SDP External Timestamp Capture Test
 *
 * Tests i210 SDP external timestamp capture of GPIO PWM signal.
 * Wire GPIO12 (PWM output) to i210 SDP0 input physically.
 *
 * Usage: sdp-capture-test -i <ifname> -g <gpio-pin>
 */

#include <argp.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../bcm2711-pwm-clock.h"
#include "../phc-utils.h"

static char ifname[16] = "eth1";
static int gpio_pin = -1;
static struct bcm2711_pwm_clock *gpio_handle;
static int ptp_fd = -1;
static clockid_t phc_clk;
static volatile int running = 1;

static struct argp_option options[] = {
	{"ifname", 'i', "IFNAME", 0, "Network interface with i210" },
	{"gpio", 'g', "PIN", 0, "GPIO pin for PWM (12, 13, 18, or 19)" },
	{ 0 }
};

static error_t parser(int key, char *arg, struct argp_state *state)
{
	switch (key) {
	case 'i':
		snprintf(ifname, sizeof(ifname), "%s", arg);
		break;
	case 'g':
		gpio_pin = atoi(arg);
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
	int res;
	uint64_t last_ts = 0;
	uint64_t pkt_count = 0;

	argp_parse(&argp, argc, argv, 0, NULL, NULL);

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	if (gpio_pin < 0) {
		fprintf(stderr, "Error: GPIO pin required (-g option)\n");
		return 1;
	}

	res = phc_open(ifname, &ptp_fd, &phc_clk, NULL);
	if (res < 0) {
		fprintf(stderr, "Failed to open PHC for %s\n", ifname);
		return 1;
	}

	res = bcm2711_pwm_clock_init(gpio_pin, &gpio_handle);
	if (res < 0) {
		fprintf(stderr, "Failed to initialize GPIO %d\n", gpio_pin);
		phc_close(ptp_fd);
		return 1;
	}

	res = bcm2711_pwm_clock_start(gpio_handle, 300);
	if (res < 0) {
		fprintf(stderr, "Failed to start PWM clock\n");
		bcm2711_pwm_clock_cleanup(gpio_handle);
		phc_close(ptp_fd);
		return 1;
	}

	fprintf(stderr, "SDP External Timestamp Capture Test - 300Hz\n");
	fprintf(stderr, "GPIO %d running at 300 Hz (expected interval: 3.333 ms)\n", gpio_pin);
	fprintf(stderr, "Make sure GPIO %d is wired to i226 SDP0 input!\n\n", gpio_pin);

	fprintf(stderr, "Configuring SDP0 pin for external timestamp input...\n");
	res = phc_pin_setfunc(ptp_fd, 0, 1, 0);
	if (res < 0) {
		fprintf(stderr, "Failed to configure SDP0 pin\n");
		bcm2711_pwm_clock_cleanup(gpio_handle);
		phc_close(ptp_fd);
		return 1;
	}

	res = phc_extts_enable(ptp_fd, 0, 1);
	if (res < 0) {
		fprintf(stderr, "Failed to enable external timestamps on SDP0\n");
		fprintf(stderr, "Check that GPIO is wired to SDP0 input\n");
		bcm2711_pwm_clock_cleanup(gpio_handle);
		phc_close(ptp_fd);
		return 1;
	}

	fprintf(stderr, "Capturing timestamps (Ctrl+C to stop)...\n\n");
	fprintf(stderr, "Packet  Timestamp (ns)         Interval (ns)  Freq (Hz)\n");
	fprintf(stderr, "------  ---------------------  -------------  ---------\n");

	while (running) {
		uint64_t ts;
		int64_t interval;
		double freq;

		res = phc_extts_read(ptp_fd, 0, &ts);
		if (res < 0) {
			if (running)
				fprintf(stderr, "Error reading timestamp\n");
			break;
		}

		pkt_count++;

		if (last_ts != 0) {
			interval = (int64_t)(ts - last_ts);
			freq = (interval > 0) ? (1e9 / interval) : 0.0;

			if (pkt_count % 100 == 0) {
				fprintf(stderr, "%6" PRIu64 "  %21" PRIu64 "  %13" PRId64 "  %9.3f\n",
					pkt_count, ts, interval, freq);
			}
		}

		last_ts = ts;
	}

	fprintf(stderr, "\nStopping...\n");
	fprintf(stderr, "Captured %" PRIu64 " timestamps\n", pkt_count);

	phc_extts_disable(ptp_fd, 0);
	bcm2711_pwm_clock_cleanup(gpio_handle);
	phc_close(ptp_fd);

	return 0;
}
