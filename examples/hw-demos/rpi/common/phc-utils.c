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

/* PTP Hardware Clock access.
 *
 * Device discovery via /sys/class/net/<ifname>/device/ptp/, TAI-UTC offset
 * from /sys/class/ptp/ptp<N>/utc_offset, clock reads through the POSIX clock
 * calls on a clockid synthesised from the device descriptor.
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

int phc_tai_offset(const char *ifname, int *tai_offset)
{
	char ptp_path[64];
	int ptp_index;

	if (!ifname || !tai_offset)
		return -1;

	if (find_ptp_device(ifname, ptp_path, sizeof(ptp_path), &ptp_index) < 0)
		return -1;

	return read_tai_offset(ptp_index, tai_offset);
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
