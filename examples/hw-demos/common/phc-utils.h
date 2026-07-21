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
 * Device discovery and clock reading for a PTP-capable network interface.
 * Everything here works on any NIC that exposes /dev/ptpN - Intel, Mellanox,
 * Broadcom, the Raspberry Pi 5's macb - and uses only sysfs and the POSIX
 * clock calls.
 *
 * External timestamp capture and pin multiplexing are not here. They live in
 * ptp-extts.h because they depend on capture channels and routable pins that
 * many NICs simply do not have, and a platform that does not use them should
 * not link them.
 */

#pragma once

#include <stdint.h>
#include <time.h>

/* Open the PHC backing a network interface.
 *
 * @ptp_fd:     receives the /dev/ptpN descriptor, close with phc_close()
 * @clkid:      receives a clockid usable with clock_gettime() and
 *              clock_nanosleep()
 * @tai_offset: receives the TAI-UTC offset in seconds, or NULL if not wanted
 *
 * Returns 0 on success, -1 if the interface has no PHC or it cannot be
 * opened. */
int phc_open(const char *ifname, int *ptp_fd, clockid_t *clkid,
	     int *tai_offset);

/* Returns 0 on success, -1 on error. */
int phc_close(int ptp_fd);

/* Read the PHC. Returns nanoseconds, or 0 on error with errno set.
 *
 * The value is on the PHC's own timescale, which for a gPTP-synchronised NIC
 * is TAI. CRF timestamps are TAI too, so the two difference directly; wall
 * clock comparisons need phc_tai_offset(). */
uint64_t phc_gettime_ns(clockid_t clkid);

/* TAI-UTC offset in seconds for the PHC backing @ifname, without opening it.
 *
 * For converting between CRF timestamps, which are TAI, and anything on
 * CLOCK_REALTIME - GPIO chardev events for instance, which the kernel stamps
 * in UTC. Read from /sys/class/ptp/ptpN/utc_offset, which reflects what the
 * PTP stack currently believes; it is not a compile-time constant and changes
 * at a leap second.
 *
 * Returns 0 on success, -1 if the interface has no PHC. On a failure to read
 * or parse the sysfs attribute, warns and yields the present-day 37. */
int phc_tai_offset(const char *ifname, int *tai_offset);
