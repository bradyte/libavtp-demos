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

/* PTP external timestamp capture and pin multiplexing. */

#include <linux/ptp_clock.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include "ptp-extts.h"

#define NSEC_PER_SEC	1000000000ULL

int ptp_pin_setfunc(int ptp_fd, unsigned int pin, unsigned int func,
		    unsigned int chan)
{
	struct ptp_pin_desc pin_desc;

	memset(&pin_desc, 0, sizeof(pin_desc));
	pin_desc.index = pin;
	pin_desc.func = func;
	pin_desc.chan = chan;

	if (ioctl(ptp_fd, PTP_PIN_SETFUNC, &pin_desc) < 0) {
		perror("PTP_PIN_SETFUNC");
		return -1;
	}

	return 0;
}

int ptp_extts_enable(int ptp_fd, unsigned int index, int rising)
{
	struct ptp_extts_request req;

	memset(&req, 0, sizeof(req));
	req.index = index;
	req.flags = PTP_ENABLE_FEATURE;
	req.flags |= rising ? PTP_RISING_EDGE : PTP_FALLING_EDGE;

	if (ioctl(ptp_fd, PTP_EXTTS_REQUEST, &req) < 0) {
		perror("PTP_EXTTS_REQUEST (enable)");
		return -1;
	}

	return 0;
}

int ptp_extts_disable(int ptp_fd, unsigned int index)
{
	struct ptp_extts_request req;

	memset(&req, 0, sizeof(req));
	req.index = index;
	req.flags = 0;

	if (ioctl(ptp_fd, PTP_EXTTS_REQUEST, &req) < 0) {
		perror("PTP_EXTTS_REQUEST (disable)");
		return -1;
	}

	return 0;
}

/* Consume one event. Events for other channels are skipped, never reported
 * as "nothing available" - a caller polling for its own channel would
 * otherwise stall on a queue holding a foreign event. */
static int extts_read_one(int ptp_fd, unsigned int channel,
			  uint64_t *timestamp_ns)
{
	struct ptp_extts_event event;
	ssize_t n;

	for (;;) {
		n = read(ptp_fd, &event, sizeof(event));
		if (n < 0) {
			perror("read extts event");
			return -1;
		}
		if (n != sizeof(event)) {
			fprintf(stderr, "Short read on extts event\n");
			return -1;
		}

		if (event.index == channel)
			break;
	}

	*timestamp_ns = (uint64_t)event.t.sec * NSEC_PER_SEC + event.t.nsec;
	return 0;
}

int ptp_extts_read(int ptp_fd, unsigned int channel, uint64_t *timestamp_ns)
{
	return extts_read_one(ptp_fd, channel, timestamp_ns);
}

int ptp_extts_read_nonblock(int ptp_fd, unsigned int channel,
			    uint64_t *timestamp_ns)
{
	struct timeval tv = { 0 };
	fd_set fds;

	FD_ZERO(&fds);
	FD_SET(ptp_fd, &fds);

	if (select(ptp_fd + 1, &fds, NULL, NULL, &tv) <= 0)
		return -1;

	return extts_read_one(ptp_fd, channel, timestamp_ns);
}
