/*
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

/* Media clock backend: stock Raspberry Pi 5.
 *
 * Signal path:
 *   RP1 pll_audio -> clk_i2s -> clk_gp0 -> GPCLK0 on GPIO4
 *   GPIO4 --jumper--> GPIO17 -> GPIO chardev rising edge, the feedback edge
 *   clk_i2s also feeds the DAC master clock
 *
 * Steering trims pll_audio's 24-bit fractional feedback divider, so the VCO
 * and everything below it move together: this is clock recovery for the whole
 * audio domain, not a correction applied to one tap.
 *
 * No custom hardware beyond a jumper. The cost is that the feedback edge is
 * timestamped by the kernel rather than a NIC, so it carries interrupt
 * latency the CM4's hardware capture does not.
 *
 * Timescale. GPIO chardev events are stamped on CLOCK_REALTIME, which is UTC,
 * while CRF timestamps are TAI. The offset is applied here so the core only
 * ever sees TAI, and it is read from the PTP stack rather than assumed - it
 * changes at a leap second.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "media-clock.h"
#include "phc-utils.h"
#include "rp1-clock.h"

#define NSEC_PER_SEC		1000000000ULL

#define DEFAULT_GPIOCHIP	"/dev/gpiochip0"
#define DEFAULT_GPIO_LINE	17

/* rp1_clock_init() reprograms the pll_audio VCO and polls for lock, but the
 * loop settles for longer than the lock bit indicates. Frequency acquisition
 * measures a phase slope over two seconds and would fold that settling into
 * its estimate, so the transient is waited out here rather than worked around
 * in the servo. */
#define VCO_SETTLE_MS		500

struct rp1_media_clock {
	struct rp1_clock *clk;
	int gpio_fd;
	int64_t tai_offset_ns;
};

static struct rp1_media_clock *to_rp1(struct media_clock *mc)
{
	return (struct rp1_media_clock *)mc;
}

static int rp1_probe(void)
{
	char compat[128];
	FILE *f = fopen("/proc/device-tree/compatible", "r");
	size_t i, n;

	if (!f)
		return 0;

	n = fread(compat, 1, sizeof(compat) - 1, f);
	fclose(f);
	compat[n] = '\0';

	/* NUL-separated list. The Pi 5's SoC is bcm2712; RP1 hangs off it. */
	for (i = 0; i < n; i += strlen(compat + i) + 1)
		if (strstr(compat + i, "bcm2712"))
			return 1;

	return 0;
}

/* Request rising-edge events on one line, stamped on CLOCK_REALTIME so the
 * TAI offset can be applied. Returns the line request fd. */
static int gpio_edge_open(const char *chip_path, unsigned int line)
{
	struct gpio_v2_line_request req = { 0 };
	int chip_fd, req_fd;

	chip_fd = open(chip_path, O_RDONLY | O_CLOEXEC);
	if (chip_fd < 0) {
		fprintf(stderr, "Failed to open %s: %s\n", chip_path,
			strerror(errno));
		return -1;
	}

	req.offsets[0] = line;
	req.num_lines = 1;
	strncpy(req.consumer, "crf-listener", sizeof(req.consumer) - 1);
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

static void rp1_close_priv(struct rp1_media_clock *c)
{
	if (!c)
		return;

	if (c->gpio_fd >= 0)
		close(c->gpio_fd);
	if (c->clk)
		rp1_clock_cleanup(c->clk);

	free(c);
}

static void rp1_close(struct media_clock *mc)
{
	rp1_close_priv(to_rp1(mc));
}

static int rp1_open(struct media_clock **out,
		    const struct media_clock_config *cfg)
{
	const char *chip = cfg->device ? cfg->device : DEFAULT_GPIOCHIP;
	unsigned int line = cfg->line ? cfg->line : DEFAULT_GPIO_LINE;
	struct rp1_media_clock *c;
	int tai_offset = 0;
	int res;

	c = calloc(1, sizeof(*c));
	if (!c)
		return -1;
	c->gpio_fd = -1;

	/* CRF timestamps are TAI, GPIO events are UTC. Read the current
	 * offset rather than assuming 37. */
	if (phc_tai_offset(cfg->ifname, &tai_offset) < 0) {
		fprintf(stderr, "Failed to read TAI-UTC offset for %s\n",
			cfg->ifname);
		goto err;
	}
	c->tai_offset_ns = (int64_t)tai_offset * (int64_t)NSEC_PER_SEC;

	fprintf(stderr, "  GPCLK0 (GPIO4) --jumper--> GPIO%u edge capture, "
		"TAI-UTC %ds\n", line, tai_offset);

	c->gpio_fd = gpio_edge_open(chip, line);
	if (c->gpio_fd < 0)
		goto err;

	res = rp1_clock_init(&c->clk);
	if (res < 0) {
		fprintf(stderr, "Failed to init RP1 clock (needs root): %s\n",
			strerror(-res));
		goto err;
	}

	res = rp1_clock_enable(c->clk, cfg->edge_hz);
	if (res < 0) {
		fprintf(stderr, "Failed to enable RP1 clocks at %.1f Hz\n",
			cfg->edge_hz);
		goto err;
	}

	fprintf(stderr, "rp1-clock: waiting %d ms for the VCO to settle\n",
		VCO_SETTLE_MS);
	usleep(VCO_SETTLE_MS * 1000);

	*out = (struct media_clock *)c;
	return 0;

err:
	rp1_close_priv(c);
	return -1;
}

static int rp1_edge_read(struct media_clock *mc, uint64_t *tai_ns,
			 int timeout_ms)
{
	struct rp1_media_clock *c = to_rp1(mc);
	struct gpio_v2_line_event event;
	ssize_t n;

	if (timeout_ms >= 0) {
		struct pollfd pfd = { .fd = c->gpio_fd, .events = POLLIN };

		if (poll(&pfd, 1, timeout_ms) <= 0)
			return -1;
	}

	n = read(c->gpio_fd, &event, sizeof(event));
	if (n < 0) {
		if (errno != EINTR)
			perror("read gpio edge event");
		return -1;
	}
	if (n != sizeof(event)) {
		fprintf(stderr, "Short read on gpio edge event\n");
		return -1;
	}

	/* Kernel stamped this on CLOCK_REALTIME. */
	*tai_ns = event.timestamp_ns + (uint64_t)c->tai_offset_ns;
	return 0;
}

static int rp1_adjust(struct media_clock *mc, double ppb)
{
	return rp1_clock_adjust(to_rp1(mc)->clk, ppb);
}

static uint64_t rp1_now_tai_ns(struct media_clock *mc)
{
	struct rp1_media_clock *c = to_rp1(mc);
	struct timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);

	return (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec +
	       (uint64_t)c->tai_offset_ns;
}

const struct media_clock_ops mc_rp1_ops = {
	.name		= "rpi5",
	.probe		= rp1_probe,
	.open		= rp1_open,
	.edge_read	= rp1_edge_read,
	.adjust		= rp1_adjust,
	.now_tai_ns	= rp1_now_tai_ns,
	.close		= rp1_close,
};
