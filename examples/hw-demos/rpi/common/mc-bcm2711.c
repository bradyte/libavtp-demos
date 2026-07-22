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

/* Media clock backend: Raspberry Pi CM4 with an Intel i226 and a CS2600.
 *
 * Signal path:
 *   BCM2711 PWM (GPIO12) -> CS2600 CLK_IN
 *   CS2600 AUX_OUT, a buffered copy of CLK_IN -> i226 SDP0, the feedback edge
 *   CS2600 CLK_OUT, 24.576 MHz -> I2S MCLK
 *
 * The CS2600 is inside the control loop, not merely downstream of it: it
 * multiplies the recovered edge rate up to MCLK and returns the buffered copy
 * that the servo measures. Steering is the PWM fractional divider.
 *
 * Both the capture and the CRF stream are timestamped by the i226 PHC, which
 * a gPTP-synchronised NIC keeps on TAI, so edges need no conversion here.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "bcm2711-pwm-clock.h"
#include "cs2600.h"
#include "media-clock.h"
#include "phc-utils.h"
#include "ptp-extts.h"

#define NSEC_PER_SEC		1000000000ULL

/* GPIO12 is PWM0 on the BCM2711; the board routes it to CS2600 CLK_IN. */
#define PWM_GPIO		12

/* I2S master clock synthesised from the recovered edge rate, 512 x 48 kHz.
 * The multiplication ratio follows from edge_hz. */
#define CS2600_MCLK_HZ		24576000.0

#define DEFAULT_I2C_DEV		"/dev/i2c-1"

/* SDP0 is pin 0 and is routed to capture channel 0. */
#define SDP_PIN			0
#define SDP_CHANNEL		0
#define PTP_PIN_FUNC_EXTTS	1

/* The PWM is enabled a whole PTP second after configuration so its phase is
 * deterministic run to run. Sleep to just short of the boundary, then spin. */
#define ENABLE_SPIN_MARGIN_NS	1000000

/* Backend-private state. media-clock.h leaves struct media_clock incomplete
 * so the core cannot reach in here; each backend names its own type and casts
 * the handle, rather than every backend defining struct media_clock
 * differently, which would make the tag incompatible across translation
 * units once a second backend exists. */
struct bcm2711_clock {
	struct bcm2711_pwm_clock *pwm;
	struct cs2600 cs2600;
	clockid_t phc_clk;
	int ptp_fd;
	bool cs2600_open;
};

static struct bcm2711_clock *to_bcm2711(struct media_clock *mc)
{
	return (struct bcm2711_clock *)mc;
}

static int bcm2711_probe(void)
{
	/* The PWM peripheral is only reachable through /dev/mem on a BCM2711,
	 * and the SoC identifies itself in the device tree. */
	char compat[128];
	FILE *f = fopen("/proc/device-tree/compatible", "r");
	size_t n;

	if (!f)
		return 0;

	n = fread(compat, 1, sizeof(compat) - 1, f);
	fclose(f);
	compat[n] = '\0';

	/* The property is a list of NUL-separated strings. */
	for (size_t i = 0; i < n; i += strlen(compat + i) + 1)
		if (strstr(compat + i, "bcm2711"))
			return 1;

	return 0;
}

/* Enable the PWM output on a PTP second boundary. */
static void pwm_enable_on_second(struct bcm2711_clock *c)
{
	uint64_t now = phc_gettime_ns(c->phc_clk);
	uint64_t target = ((now / NSEC_PER_SEC) + 1) * NSEC_PER_SEC;
	uint64_t wait_ns = (target - ENABLE_SPIN_MARGIN_NS) - now;
	struct timespec dur = {
		.tv_sec = wait_ns / NSEC_PER_SEC,
		.tv_nsec = wait_ns % NSEC_PER_SEC,
	};

	fprintf(stderr, "PWM: Waiting for PTP second %lu to enable...\n",
		(unsigned long)(target / NSEC_PER_SEC));

	clock_nanosleep(CLOCK_MONOTONIC, 0, &dur, NULL);
	while (phc_gettime_ns(c->phc_clk) < target)
		;

	bcm2711_pwm_clock_enable(c->pwm);

	fprintf(stderr, "PWM: Enabled at PTP +%ldns from target\n",
		(long)(phc_gettime_ns(c->phc_clk) - target));
}

static void bcm2711_close_priv(struct bcm2711_clock *c)
{
	if (!c)
		return;

	if (c->ptp_fd >= 0) {
		ptp_extts_disable(c->ptp_fd, SDP_CHANNEL);
		phc_close(c->ptp_fd);
	}
	if (c->cs2600_open)
		cs2600_close(&c->cs2600);
	if (c->pwm)
		bcm2711_pwm_clock_cleanup(c->pwm);

	free(c);
}

static void bcm2711_close(struct media_clock *mc)
{
	bcm2711_close_priv(to_bcm2711(mc));
}

static int bcm2711_open(struct media_clock **out,
			const struct media_clock_config *cfg)
{
	const char *i2c_dev = cfg->device ? cfg->device : DEFAULT_I2C_DEV;
	struct bcm2711_clock *c;
	int res;

	c = calloc(1, sizeof(*c));
	if (!c)
		return -1;
	c->ptp_fd = -1;

	res = phc_open(cfg->ifname, &c->ptp_fd, &c->phc_clk, NULL);
	if (res < 0) {
		fprintf(stderr, "Failed to open PHC for %s\n", cfg->ifname);
		goto err;
	}

	fprintf(stderr, "  GPIO %d: %.0f Hz -> CS2600 CLK_IN -> AUX_OUT -> "
		"i226 SDP%d\n", PWM_GPIO, cfg->edge_hz, SDP_PIN);

	res = bcm2711_pwm_clock_init(PWM_GPIO, &c->pwm);
	if (res < 0) {
		fprintf(stderr, "Failed to init PWM\n");
		goto err;
	}

	res = bcm2711_pwm_clock_prepare(c->pwm, (unsigned int)cfg->edge_hz);
	if (res < 0) {
		fprintf(stderr, "Failed to prepare PWM\n");
		goto err;
	}

	pwm_enable_on_second(c);

	res = cs2600_open(i2c_dev, CS2600_I2C_ADDR, &c->cs2600);
	if (res < 0) {
		fprintf(stderr, "Failed to open CS2600 on %s\n", i2c_dev);
		goto err;
	}
	c->cs2600_open = true;

	if (cs2600_check_id(&c->cs2600) < 0) {
		fprintf(stderr, "CS2600 not found (bad device ID)\n");
		goto err;
	}

	fprintf(stderr, "CS2600: Configuring %.0f Hz -> %.3f MHz (ratio %.1f)...\n",
		cfg->edge_hz, CS2600_MCLK_HZ / 1e6,
		CS2600_MCLK_HZ / cfg->edge_hz);

	res = cs2600_init_mult(&c->cs2600, CS2600_MCLK_HZ, cfg->edge_hz);
	if (res == -2) {
		fprintf(stderr, "CS2600: Frequency lock timeout\n");
		goto err;
	} else if (res < 0) {
		fprintf(stderr, "CS2600: Init failed\n");
		goto err;
	}
	fprintf(stderr, "CS2600: Locked\n");

	/* Route SDP0 to capture channel 0 and arm it for rising edges. */
	ptp_extts_disable(c->ptp_fd, SDP_CHANNEL);
	res = ptp_pin_setfunc(c->ptp_fd, SDP_PIN, PTP_PIN_FUNC_EXTTS,
			      SDP_CHANNEL);
	if (res < 0) {
		fprintf(stderr, "Failed to configure SDP%d\n", SDP_PIN);
		goto err;
	}
	res = ptp_extts_enable(c->ptp_fd, SDP_CHANNEL, 1);
	if (res < 0) {
		fprintf(stderr, "Failed to enable SDP%d capture\n", SDP_PIN);
		goto err;
	}

	*out = (struct media_clock *)c;
	return 0;

err:
	bcm2711_close_priv(c);
	return -1;
}

static int bcm2711_edge_read(struct media_clock *mc, uint64_t *tai_ns,
			     int timeout_ms)
{
	struct bcm2711_clock *c = to_bcm2711(mc);

	/* PHC timestamps are already TAI, so no conversion. */
	return ptp_extts_read_timeout(c->ptp_fd, SDP_CHANNEL, tai_ns,
				      timeout_ms);
}

static int bcm2711_adjust(struct media_clock *mc, double ppb)
{
	return bcm2711_pwm_clock_adjust_freq(to_bcm2711(mc)->pwm, ppb);
}

static uint64_t bcm2711_now_tai_ns(struct media_clock *mc)
{
	return phc_gettime_ns(to_bcm2711(mc)->phc_clk);
}

/* The PWM clock manager's 12-bit fractional divider code. One LSB is about
 * 800 ppb, so this is the loop's quantisation floor made visible. */
static uint32_t bcm2711_actuator_code(struct media_clock *mc)
{
	return bcm2711_pwm_clock_get_divf(to_bcm2711(mc)->pwm);
}

const struct media_clock_ops mc_bcm2711_ops = {
	.name		= "cm4",
	.probe		= bcm2711_probe,
	.open		= bcm2711_open,
	.edge_read	= bcm2711_edge_read,
	.adjust		= bcm2711_adjust,
	.now_tai_ns	= bcm2711_now_tai_ns,
	.actuator_code	= bcm2711_actuator_code,
	.close		= bcm2711_close,
};
