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

/* Media clock backend.
 *
 * A CRF listener is the same program on every board: receive timestamps,
 * pair each with a feedback edge, feed the phase error to a servo, apply the
 * correction. Only two things are platform-specific, and this is the seam
 * between them:
 *
 *   where feedback edges come from    i226 external timestamp capture on the
 *                                     CM4; GPIO chardev edge events on a
 *                                     Raspberry Pi 5
 *   how the clock is steered          BCM2711 PWM fractional divider feeding
 *                                     a CS2600 multiplier; RP1 pll_audio
 *                                     FBDIV_FRAC
 *
 * Two rules keep the core free of platform knowledge.
 *
 * Edge timestamps are always returned on the same timescale as the CRF
 * stream, which is TAI. A backend whose capture hardware stamps in anything
 * else converts internally - the Pi 5 GPIO chardev stamps CLOCK_REALTIME, so
 * that backend applies the TAI-UTC offset itself. The core only ever
 * subtracts two numbers on one timescale.
 *
 * The output rate is a parameter, not a constant. open() is told the edge
 * frequency the stream implies, derived from the CRF profile, and either
 * configures hardware to produce it or fails. Nothing downstream hardcodes
 * 300 Hz.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

struct media_clock;

struct media_clock_config {
	/* Interface whose PHC provides the timescale, and on the CM4 also the
	 * capture hardware. */
	const char *ifname;

	/* Feedback edges per second the backend must generate, from
	 * crf_profile_edge_hz(). One edge per CRF timestamp. */
	double edge_hz;

	/* Backend-specific device path: I2C bus for the CS2600, GPIO chardev
	 * for the Pi 5. NULL selects the backend's default. */
	const char *device;

	/* Backend-specific line or pin selector. */
	unsigned int line;
};

struct media_clock_ops {
	const char *name;

	/* Non-zero if this backend matches the hardware it is running on.
	 * Used to pick a default when none is named. */
	int (*probe)(void);

	/* Bring up capture and clock generation at cfg->edge_hz. On failure
	 * nothing is left allocated or configured. */
	int (*open)(struct media_clock **mc,
		    const struct media_clock_config *cfg);

	/* Next feedback edge, in TAI nanoseconds.
	 *
	 * @timeout_ms: 0 polls, a positive value waits that long, negative
	 *              blocks indefinitely.
	 *
	 * Returns 0 on success, -1 if no edge arrived in time or on error.
	 * A timeout is normal - the caller counts it as a missed edge. */
	int (*edge_read)(struct media_clock *mc, uint64_t *tai_ns,
			 int timeout_ms);

	/* Apply a frequency correction in parts per billion. Positive speeds
	 * the clock up. Returns 0 on success. */
	int (*adjust)(struct media_clock *mc, double ppb);

	/* Current time on the backend's timescale, TAI nanoseconds. Used for
	 * transit reporting; 0 if the backend has no clock of its own. */
	uint64_t (*now_tai_ns)(struct media_clock *mc);

	/* Raw value last written to the frequency actuator, for logging. Its
	 * meaning is backend-specific - a PWM fractional divider code, a PLL
	 * feedback fraction - so it is only comparable against itself. Its
	 * resolution is the quantisation floor of the whole loop, which makes
	 * it worth recording. NULL if the backend has nothing to report. */
	uint32_t (*actuator_code)(struct media_clock *mc);

	void (*close)(struct media_clock *mc);
};

/* Select a backend by name, or NULL to probe. Returns NULL if the name is
 * unknown or no backend claims the hardware. */
const struct media_clock_ops *media_clock_select(const char *name);

/* Space-separated list of compiled-in backend names, for usage text. */
const char *media_clock_names(void);
