/*
 * CRF Listener — Raspberry Pi 5 (GPIO chardev edge capture)
 *
 * Receives IEEE 1722 CRF timestamps and recovers the media clock using a
 * PI servo at 300 Hz. Feedback comes from GPIO chardev edge detection on
 * the GPCLK0 output (looped back via jumper GPIO4 → GPIO17).
 *
 * Signal path:
 *   RP1 pll_audio → clk_i2s (3.072 MHz) → clk_gp0 ÷10240 → GPIO4 (300 Hz)
 *   GPIO4 --jumper--> GPIO17 → GPIO chardev rising edge → CLOCK_REALTIME ts
 *
 * Each CRF timestamp has exactly one feedback edge from the GPCLK output.
 * The PI servo computes the frequency adjustment to lock phase.
 *
 * Usage:
 *   sudo chrt -f 80 taskset -c 2 ./crf-listener-rpi \
 *       -i eth0 -d 91:E0:F0:00:FE:00 -g /dev/gpiochip0 -p 17
 */

#include <argp.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/gpio.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/if_packet.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "crf-common.h"
#include "crf-receiver.h"
#include "pi-hw.h"
#include "clock-adjust.h"
#include "rp1-clock.h"

#define SERVO_MAX_PPB		2000000.0
#define SERVO_STEP_THRESH	100000000.0

#define EDGE_FREQ_HZ		300
#define NSEC_PER_SEC		1000000000ULL

/* TAI-UTC offset (leap seconds). CRF timestamps are TAI;
 * GPIO edge timestamps are CLOCK_REALTIME (UTC). */
#define TAI_UTC_OFFSET_S	37

/* Tolerance: ±half of one 300 Hz period (~1.67 ms) */
#define EDGE_TOLERANCE_NS	(NSEC_PER_SEC / (EDGE_FREQ_HZ * 2))

static char ifname[IFNAMSIZ];
static uint8_t crf_macaddr[ETH_ALEN];
static char gpiochip_path[64] = "/dev/gpiochip0";
static unsigned int gpio_line = 17;

static struct pi_servo *servo;
static struct crf_receiver *crf_rx;
static int gpio_edge_fd = -1;

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

/* CSV logging */
static FILE *csv_log;
static bool verbose;

static struct argp_option options[] = {
	{"dst-addr", 'd', "MACADDR", 0, "Stream Destination MAC address"},
	{"ifname", 'i', "IFNAME", 0, "Network Interface"},
	{"gpiochip", 'g', "PATH", 0, "GPIO chip device (default /dev/gpiochip0)"},
	{"pin", 'p', "NUM", 0, "GPIO line for edge capture (default 17)"},
	{"log", 'l', "FILE", 0, "CSV log file"},
	{"verbose", 'v', NULL, 0, "Print per-edge phase error"},
	{0}
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
	case 'g':
		strncpy(gpiochip_path, arg, sizeof(gpiochip_path) - 1);
		break;
	case 'p':
		gpio_line = atoi(arg);
		break;
	case 'l':
		csv_log = fopen(arg, "w");
		if (!csv_log) {
			fprintf(stderr, "Failed to open log file: %s\n", arg);
			exit(EXIT_FAILURE);
		}
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
	(void)signum;
	running = false;
}

/*
 * Open GPIO chardev and request rising edge detection on the specified line.
 * Returns the line request fd (supports poll/read for edge events).
 */
static int gpio_edge_open(const char *chip_path, unsigned int line)
{
	int chip_fd, req_fd;
	struct gpio_v2_line_request req = {0};

	chip_fd = open(chip_path, O_RDONLY | O_CLOEXEC);
	if (chip_fd < 0) {
		fprintf(stderr, "Failed to open %s: %s\n",
			chip_path, strerror(errno));
		return -1;
	}

	req.offsets[0] = line;
	req.num_lines = 1;
	strncpy(req.consumer, "crf-listener-rpi", sizeof(req.consumer) - 1);
	req.config.flags = GPIO_V2_LINE_FLAG_INPUT |
			   GPIO_V2_LINE_FLAG_EDGE_RISING |
			   GPIO_V2_LINE_FLAG_BIAS_DISABLED |
			   GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME;

	if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
		fprintf(stderr, "GPIO_V2_GET_LINE_IOCTL failed for line %u: %s\n",
			line, strerror(errno));
		close(chip_fd);
		return -1;
	}

	req_fd = req.fd;
	close(chip_fd);
	return req_fd;
}

/*
 * Read one rising edge event (blocking).
 * Returns 0 on success with timestamp in *timestamp_ns.
 */
static int gpio_edge_read(int fd, uint64_t *timestamp_ns)
{
	struct gpio_v2_line_event event;
	ssize_t n;

	n = read(fd, &event, sizeof(event));
	if (n < 0) {
		if (errno != EINTR)
			perror("read gpio edge event");
		return -1;
	}
	if (n != sizeof(event)) {
		fprintf(stderr, "Short read on gpio edge event\n");
		return -1;
	}

	*timestamp_ns = event.timestamp_ns;
	return 0;
}

/*
 * Non-blocking read of one edge event.
 * Returns 0 on success, -1 if nothing ready.
 */
static int gpio_edge_read_nonblock(int fd, uint64_t *timestamp_ns)
{
	struct pollfd pfd = { .fd = fd, .events = POLLIN };

	if (poll(&pfd, 1, 0) <= 0)
		return -1;

	return gpio_edge_read(fd, timestamp_ns);
}

/*
 * Synchronize: discard edges older than crf_ts, keep first one within tolerance.
 */
static bool synchronize(uint64_t crf_ts)
{
	uint64_t edge_ts;

	while (gpio_edge_read_nonblock(gpio_edge_fd, &edge_ts) == 0) {
		int64_t delta = (int64_t)(edge_ts - crf_ts);

		if (llabs(delta) <= (int64_t)EDGE_TOLERANCE_NS) {
			fprintf(stderr, "Synchronized: delta=%+" PRId64 "ns\n",
				delta);
			return true;
		}

		if (delta > 0)
			break;
	}

	return false;
}

static void on_crf_timestamps(uint64_t *timestamps, int count, uint8_t seq,
			      void *ctx)
{
	(void)ctx;
	(void)seq;

	for (int i = 0; i < count; i++) {
		uint64_t crf_utc = timestamps[i] -
				   (uint64_t)TAI_UTC_OFFSET_S * NSEC_PER_SEC;

		if (!synchronized) {
			if (!synchronize(crf_utc))
				continue;
			synchronized = true;
			continue;
		}

		uint64_t edge_ts;
		int res;

		res = gpio_edge_read(gpio_edge_fd, &edge_ts);
		if (res < 0) {
			edge_miss_count++;
			continue;
		}

		last_edge_ts = edge_ts;

		int64_t phase_error = (int64_t)(edge_ts - crf_utc);

		if (llabs(phase_error) > (int64_t)EDGE_TOLERANCE_NS) {
			edge_miss_count++;
			synchronized = false;
			continue;
		}

		enum servo_state state;
		double ppb = pi_servo_sample(servo, phase_error, edge_ts, &state);

		if (state != SERVO_UNLOCKED && clock_adjust)
			clock_adjust(clock_adjust_ctx, ppb);

		servo_update_count++;

		if (verbose)
			fprintf(stdout, "phase_ns=%+" PRId64 " ppb=%+.1f state=%d\n",
				phase_error, ppb, state);

		if (servo_update_count <= 10 || servo_update_count % 300 == 0) {
			fprintf(stderr, "upd=%6" PRIu64 "  phase=%+8" PRId64
				"ns  ppb=%+9.1f  state=%d\n",
				servo_update_count, phase_error, ppb, state);
		}

		if (csv_log) {
			fprintf(csv_log, "%" PRIu64 ",%" PRId64 ",%.3f,%.3f,%d\n",
				servo_update_count, phase_error, ppb,
				pi_servo_get_drift(servo), state);
			fflush(csv_log);
		}
	}
}

static void on_crf_drop(uint8_t expected, uint8_t actual, void *ctx)
{
	(void)ctx;
	uint8_t gap = actual - expected;

	pkt_dropped += gap;

	/* Drain stale edges that accumulated during the gap */
	unsigned int stale = gap * CRF_TIMESTAMPS_PER_PKT;
	for (unsigned int i = 0; i < stale; i++) {
		uint64_t discard;
		if (gpio_edge_read_nonblock(gpio_edge_fd, &discard) < 0)
			break;
	}

	fprintf(stderr, "WARNING: Dropped %d pkt(s) (seq %d→%d)\n",
		gap, expected, actual);
}

static struct rp1_clock *rp1_clk;

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

	setlinebuf(stderr);
	setlinebuf(stdout);
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
		perror("mlockall");
		return 1;
	}

	fprintf(stderr, "CRF Listener — RPi5 GPIO Edge Capture (300 Hz servo)\n");
	fprintf(stderr, "  GPCLK0 (GPIO4) --jumper--> GPIO%u edge detect\n", gpio_line);
	fprintf(stderr, "  Servo: %d Hz (1:1 CRF-to-edge)\n\n", EDGE_FREQ_HZ);

	/* Open GPIO chardev for edge capture */
	gpio_edge_fd = gpio_edge_open(gpiochip_path, gpio_line);
	if (gpio_edge_fd < 0) {
		fprintf(stderr, "Failed to open GPIO edge capture\n");
		return 1;
	}

	/* RP1 clock manager — enable clk_gp0 and steer divider */
	res = rp1_clock_init(&rp1_clk);
	if (res < 0) {
		fprintf(stderr, "Failed to init RP1 clock (need root): %s\n",
			strerror(-res));
		goto err_gpio;
	}

	res = rp1_clock_enable(rp1_clk);
	if (res < 0) {
		fprintf(stderr, "Failed to enable RP1 clocks\n");
		goto err_rp1;
	}

	clock_adjust = rp1_clock_adjust_fn;
	clock_adjust_ctx = rp1_clk;

	servo = pi_servo_create(SERVO_MAX_PPB, SERVO_STEP_THRESH);
	if (!servo) {
		fprintf(stderr, "Failed to create servo\n");
		goto err_rp1;
	}

	/* VCO was just reprogrammed — frequency estimation during the transient
	 * produces a bogus drift. Start at drift=0, let the integrator find it. */
	pi_servo_skip_freq_est(servo);
	fprintf(stderr, "Servo: skipped freq est, drift=0, default gains\n");

	if (csv_log) {
		fprintf(csv_log, "servo_update,phase_error_ns,ppb,drift,servo_state\n");
		fflush(csv_log);
	}

	/* macb driver doesn't honor AF_PACKET multicast MAC bind filtering.
	 * Create SOCK_DGRAM bound to interface+protocol only, enable promisc. */
	{
		struct sockaddr_ll sk_addr = {0};
		struct ifreq req;
		struct packet_mreq mreq = {0};

		crf_fd = socket(AF_PACKET, SOCK_DGRAM, htons(ETH_P_TSN));
		if (crf_fd < 0) {
			perror("socket");
			goto err_servo;
		}

		strncpy(req.ifr_name, ifname, IFNAMSIZ - 1);
		if (ioctl(crf_fd, SIOCGIFINDEX, &req) < 0) {
			perror("SIOCGIFINDEX");
			close(crf_fd);
			goto err_servo;
		}

		sk_addr.sll_family = AF_PACKET;
		sk_addr.sll_protocol = htons(ETH_P_TSN);
		sk_addr.sll_ifindex = req.ifr_ifindex;

		if (bind(crf_fd, (struct sockaddr *)&sk_addr, sizeof(sk_addr)) < 0) {
			perror("bind");
			close(crf_fd);
			goto err_servo;
		}

		mreq.mr_ifindex = sk_addr.sll_ifindex;
		mreq.mr_type = PACKET_MR_PROMISC;
		if (setsockopt(crf_fd, SOL_PACKET, PACKET_ADD_MEMBERSHIP,
			       &mreq, sizeof(mreq)) < 0) {
			perror("PACKET_MR_PROMISC");
			close(crf_fd);
			goto err_servo;
		}
	}

	crf_rx = crf_receiver_create_from_fd(crf_fd, on_crf_timestamps,
					     on_crf_drop, NULL);
	if (!crf_rx) {
		fprintf(stderr, "Failed to create CRF receiver\n");
		close(crf_fd);
		goto err_servo;
	}

	fprintf(stderr, "Waiting for CRF stream on %s (fd=%d)...\n", ifname, crf_fd);
	fflush(stderr);

	while (running) {
		fd_set fds;
		struct timeval tv = {.tv_sec = 1};

		FD_ZERO(&fds);
		FD_SET(crf_fd, &fds);

		res = select(crf_fd + 1, &fds, NULL, NULL, &tv);
		if (res < 0) {
			if (errno == EINTR)
				break;
			perror("select");
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

	crf_receiver_destroy(crf_rx);
err_servo:
	pi_servo_destroy(servo);
err_rp1:
	rp1_clock_cleanup(rp1_clk);
err_gpio:
	close(gpio_edge_fd);
	return 0;
}
