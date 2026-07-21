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

/* Linux PTP Hardware Clock (PHC) Utilities
 *
 * Thin wrappers around Linux kernel PTP/PHC API (linux/ptp_clock.h).
 * Uses standard sysfs + ioctl interfaces that work with any PHC-capable NIC.
 *
 * Implementation notes:
 * - Device discovery via /sys/class/net/<ifname>/device/ptp/
 * - TAI-UTC offset from /sys/class/ptp/ptp<N>/utc_offset
 * - Clock access via posix clock_gettime()/clock_nanosleep()
 * - External timestamp via PTP_EXTTS_REQUEST ioctl
 * - Pin config via PTP_PIN_SETFUNC ioctl (hardware-dependent)
 */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/ptp_clock.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "phc-utils.h"

#define NSEC_PER_SEC	1000000000ULL

#define FD_TO_CLOCKID(fd)	((~(clockid_t)(fd) << 3) | 3)
#define CLOCKID_TO_FD(clk)	((unsigned int) ~((clk) >> 3))

static int find_ptp_device(const char *ifname, char *ptp_path, size_t path_len,
				int *ptp_index)
{
	char path[256];
	DIR *dir;
	struct dirent *entry;

	*ptp_index = -1;

	snprintf(path, sizeof(path), "/sys/class/net/%s/device/ptp", ifname);

	dir = opendir(path);
	if (!dir) {
		fprintf(stderr, "Failed to open %s: %s\n", path, strerror(errno));
		fprintf(stderr, "Interface %s may not have PTP support\n", ifname);
		return -1;
	}

	while ((entry = readdir(dir)) != NULL) {
		if (sscanf(entry->d_name, "ptp%d", ptp_index) == 1) {
			snprintf(ptp_path, path_len, "/dev/ptp%d", *ptp_index);
			closedir(dir);
			return 0;
		}
	}

	closedir(dir);
	fprintf(stderr, "No PTP device found for interface %s\n", ifname);
	return -1;
}

static int read_tai_offset(int ptp_index, int *tai_offset)
{
	char path[256];
	FILE *f;
	int offset;

	if (!tai_offset)
		return -1;

	snprintf(path, sizeof(path), "/sys/class/ptp/ptp%d/utc_offset", ptp_index);

	f = fopen(path, "r");
	if (!f) {
		fprintf(stderr, "Warning: Failed to read %s: %s\n", path, strerror(errno));
		fprintf(stderr, "Defaulting to TAI-UTC offset of 37 seconds\n");
		*tai_offset = 37;
		return 0;
	}

	if (fscanf(f, "%d", &offset) != 1) {
		fprintf(stderr, "Warning: Failed to parse TAI offset from %s\n", path);
		fclose(f);
		*tai_offset = 37;
		return 0;
	}

	fclose(f);
	*tai_offset = offset;
	return 0;
}

int phc_open(const char *ifname, int *ptp_fd, clockid_t *clkid, int *tai_offset)
{
	char ptp_path[64];
	int fd, ptp_index;

	if (!ifname || !ptp_fd || !clkid)
		return -1;

	if (find_ptp_device(ifname, ptp_path, sizeof(ptp_path), &ptp_index) < 0)
		return -1;

	fd = open(ptp_path, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Failed to open %s: %s\n", ptp_path, strerror(errno));
		return -1;
	}

	*ptp_fd = fd;
	*clkid = FD_TO_CLOCKID(fd);

	if (tai_offset) {
		read_tai_offset(ptp_index, tai_offset);
		printf("Opened PTP device: %s (fd=%d, clkid=%d, TAI-UTC=%ds)\n",
			ptp_path, fd, *clkid, *tai_offset);
	} else {
		printf("Opened PTP device: %s (fd=%d, clkid=%d)\n",
			ptp_path, fd, *clkid);
	}

	return 0;
}

int phc_close(int ptp_fd)
{
	if (ptp_fd < 0)
		return -1;

	return close(ptp_fd);
}

uint64_t phc_gettime_ns(clockid_t clkid)
{
	struct timespec ts;

	if (clock_gettime(clkid, &ts) < 0) {
		perror("Failed to get PHC time");
		return 0;
	}

	return (ts.tv_sec * NSEC_PER_SEC) + ts.tv_nsec;
}

int phc_sleep_until(clockid_t clkid, uint64_t target_ns)
{
	struct timespec ts;

	ts.tv_sec = target_ns / NSEC_PER_SEC;
	ts.tv_nsec = target_ns % NSEC_PER_SEC;

	clock_nanosleep(clkid, TIMER_ABSTIME, &ts, NULL);
	return 0;
}

uint64_t timespec_to_ns(const struct timespec *ts)
{
	if (!ts)
		return 0;

	return (ts->tv_sec * NSEC_PER_SEC) + ts->tv_nsec;
}

void ns_to_timespec(uint64_t ns, struct timespec *ts)
{
	if (!ts)
		return;

	ts->tv_sec = ns / NSEC_PER_SEC;
	ts->tv_nsec = ns % NSEC_PER_SEC;
}

int phc_extts_enable(int ptp_fd, unsigned int index, int rising)
{
	struct ptp_extts_request extts_req;

	memset(&extts_req, 0, sizeof(extts_req));
	extts_req.index = index;
	extts_req.flags = PTP_ENABLE_FEATURE;
	extts_req.flags |= rising ? PTP_RISING_EDGE : PTP_FALLING_EDGE;

	if (ioctl(ptp_fd, PTP_EXTTS_REQUEST, &extts_req) < 0) {
		perror("PTP_EXTTS_REQUEST (enable)");
		return -1;
	}

	return 0;
}

int phc_extts_disable(int ptp_fd, unsigned int index)
{
	struct ptp_extts_request extts_req;

	memset(&extts_req, 0, sizeof(extts_req));
	extts_req.index = index;
	extts_req.flags = 0;

	if (ioctl(ptp_fd, PTP_EXTTS_REQUEST, &extts_req) < 0) {
		perror("PTP_EXTTS_REQUEST (disable)");
		return -1;
	}

	return 0;
}

int phc_extts_read(int ptp_fd, unsigned int channel, uint64_t *timestamp_ns)
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

int phc_pin_setfunc(int ptp_fd, unsigned int pin, unsigned int func, unsigned int chan)
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

int phc_perout_oneshot(int ptp_fd, unsigned int channel,
		       uint64_t start_ns, uint64_t pulse_ns)
{
	struct ptp_perout_request req;

	memset(&req, 0, sizeof(req));
	req.index = channel;
	req.start.sec = start_ns / NSEC_PER_SEC;
	req.start.nsec = start_ns % NSEC_PER_SEC;

	/* Try one-shot first (kernel 5.8+) */
	req.flags = PTP_PEROUT_ONE_SHOT | PTP_PEROUT_DUTY_CYCLE;
	req.period.sec = 0;
	req.period.nsec = 0;
	req.on.sec = pulse_ns / NSEC_PER_SEC;
	req.on.nsec = pulse_ns % NSEC_PER_SEC;

	if (ioctl(ptp_fd, PTP_PEROUT_REQUEST2, &req) == 0)
		return 0;

	/* Fallback: periodic with long period, caller must disable after pulse */
	memset(&req, 0, sizeof(req));
	req.index = channel;
	req.flags = PTP_PEROUT_DUTY_CYCLE;
	req.start.sec = start_ns / NSEC_PER_SEC;
	req.start.nsec = start_ns % NSEC_PER_SEC;
	req.period.sec = 1;
	req.period.nsec = 0;
	req.on.sec = pulse_ns / NSEC_PER_SEC;
	req.on.nsec = pulse_ns % NSEC_PER_SEC;

	if (ioctl(ptp_fd, PTP_PEROUT_REQUEST2, &req) == 0)
		return 1;  /* caller must disable */

	/* Last resort: basic perout without duty cycle (50% duty, 1Hz) */
	memset(&req, 0, sizeof(req));
	req.index = channel;
	req.flags = 0;
	req.start.sec = start_ns / NSEC_PER_SEC;
	req.start.nsec = start_ns % NSEC_PER_SEC;
	req.period.sec = 1;
	req.period.nsec = 0;

	if (ioctl(ptp_fd, PTP_PEROUT_REQUEST, &req) < 0) {
		perror("PTP_PEROUT_REQUEST (fallback)");
		return -1;
	}

	return 1;  /* caller must disable */
}

int phc_perout_disable(int ptp_fd, unsigned int channel)
{
	struct ptp_perout_request req;

	memset(&req, 0, sizeof(req));
	req.index = channel;

	if (ioctl(ptp_fd, PTP_PEROUT_REQUEST, &req) < 0) {
		perror("PTP_PEROUT_REQUEST (disable)");
		return -1;
	}

	return 0;
}
