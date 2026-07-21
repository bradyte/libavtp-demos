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
 * Thin wrappers around Linux kernel PHC API for PTP-capable network interfaces.
 * Works with any NIC that exposes /dev/ptpN (Intel, Mellanox, Broadcom, etc.).
 *
 * HARDWARE PORTABILITY:
 * ======================
 * GENERIC functions (work with any PHC-capable NIC):
 *   - phc_open/close         - Device discovery and lifecycle
 *   - phc_gettime_ns         - Read hardware timestamp
 *   - phc_sleep_until        - Sleep until absolute PHC time
 *   - timespec conversion    - Pure utility functions
 *
 * HARDWARE-SPECIFIC functions (require NIC-specific knowledge):
 *   - phc_extts_*            - External timestamp capture
 *   - phc_pin_setfunc        - Pin function multiplexing
 *
 * Pin numbering, capabilities, and naming are NIC-specific:
 *   Intel i210:  SDP0 (pin 0), SDP1 (pin 1)
 *   Intel i226:  Different pin mapping
 *   Other NICs:  May not support extts or have different pin counts
 *
 * CALLER RESPONSIBILITY: Application must know its hardware's pin configuration.
 *
 * Example (i210-specific):
 *   phc_pin_setfunc(fd, 0, 1, 0);  // i210 SDP0 → extts channel 0
 *
 * For true hardware abstraction, create NIC-specific wrapper (e.g., i210-extts.c).
 */

#pragma once

#include <stdint.h>
#include <time.h>

/*============================================================================
 * GENERIC PHC FUNCTIONS (hardware-agnostic)
 *============================================================================*/

/* Open PTP Hardware Clock for the specified network interface.
 * @ifname: Network interface name (e.g., "eth1")
 * @ptp_fd: Pointer to store the PTP device file descriptor
 * @clkid: Pointer to store the clockid for use with clock_gettime/clock_nanosleep
 * @tai_offset: Pointer to store TAI-UTC offset in seconds (NULL if not needed)
 *
 * Returns:
 *    0: Success.
 *   -1: Failed to open PHC.
 */
int phc_open(const char *ifname, int *ptp_fd, clockid_t *clkid, int *tai_offset);

/* Close PTP Hardware Clock.
 * @ptp_fd: PTP device file descriptor from phc_open()
 *
 * Returns:
 *    0: Success.
 *   -1: Error closing device.
 */
int phc_close(int ptp_fd);

/* Get current time from PTP Hardware Clock.
 * @clkid: Clock ID from phc_open()
 *
 * Returns:
 *    Time in nanoseconds.
 *    0 on error (caller should check errno).
 */
uint64_t phc_gettime_ns(clockid_t clkid);

/* Sleep until absolute PTP time.
 * @clkid: Clock ID from phc_open()
 * @target_ns: Absolute PTP time in nanoseconds to wake at
 *
 * Returns:
 *    0: Success.
 *   -1: Failed to sleep.
 */
int phc_sleep_until(clockid_t clkid, uint64_t target_ns);

/* Convert timespec to nanoseconds.
 * @ts: Pointer to timespec
 *
 * Returns:
 *    Time in nanoseconds.
 */
uint64_t timespec_to_ns(const struct timespec *ts);

/* Convert nanoseconds to timespec.
 * @ns: Time in nanoseconds
 * @ts: Pointer to timespec to fill
 */
void ns_to_timespec(uint64_t ns, struct timespec *ts);

/*============================================================================
 * HARDWARE-SPECIFIC FUNCTIONS (caller must know NIC pin configuration)
 *============================================================================*/

/* Enable external timestamp capture on PHC.
 *
 * HARDWARE-SPECIFIC: Pin index and capabilities vary by NIC.
 *
 * @ptp_fd: PTP device file descriptor from phc_open()
 * @index: Hardware-specific pin/channel index
 *         Intel i210: 0=SDP0, 1=SDP1
 *         Intel i226: Different mapping
 *         Other NICs: Consult datasheet
 * @rising: 1 to capture rising edges, 0 for falling edges
 *          Note: i210 only supports double-edge capture in hardware
 *
 * Returns:
 *    0: Success.
 *   -1: Failed to enable capture (check dmesg for driver errors).
 */
int phc_extts_enable(int ptp_fd, unsigned int index, int rising);

/* Disable external timestamp capture on PHC.
 *
 * @ptp_fd: PTP device file descriptor from phc_open()
 * @index: Hardware-specific pin/channel index (must match phc_extts_enable)
 *
 * Returns:
 *    0: Success.
 *   -1: Failed to disable capture.
 */
int phc_extts_disable(int ptp_fd, unsigned int index);

/* Read next external timestamp event (blocking).
 * Filters by channel index, discarding events from other channels.
 * @ptp_fd: PTP device file descriptor from phc_open()
 * @channel: Only return events matching this channel index
 * @timestamp_ns: Pointer to store captured timestamp in nanoseconds
 *
 * Returns:
 *    0: Success, timestamp captured.
 *   -1: Error reading event.
 */
int phc_extts_read(int ptp_fd, unsigned int channel, uint64_t *timestamp_ns);

/* Configure PHC pin function (mux pin to specific capability).
 *
 * HARDWARE-SPECIFIC: Pin numbering is NIC-specific. Many NICs do not
 * support runtime pin configuration (fixed function pins).
 *
 * @ptp_fd: PTP device file descriptor from phc_open()
 * @pin: Hardware-specific physical pin index
 *       Intel i210: 0=SDP0, 1=SDP1, 2=U.FL1, 3=U.FL2
 *       Intel i226: Different mapping
 *       Other NICs: May not support PTP_PIN_SETFUNC ioctl
 * @func: Pin function (0=none, 1=extts, 2=perout, 3=pps)
 * @chan: Logical channel index to associate with this pin (usually matches pin)
 *
 * Example (i210): Configure SDP0 for external timestamp capture
 *   phc_pin_setfunc(fd, 0, 1, 0);  // pin 0 (SDP0), func 1 (extts), chan 0
 *
 * Returns:
 *    0: Success.
 *   -1: Failed to configure pin (not all NICs support this).
 */
int phc_pin_setfunc(int ptp_fd, unsigned int pin, unsigned int func, unsigned int chan);

/* Schedule a one-shot pulse on a perout channel at an absolute PTP time.
 * The pulse goes high at start_ns for pulse_ns duration.
 * Requires PTP_PEROUT_ONE_SHOT support (kernel 5.8+).
 *
 * @ptp_fd: PTP device file descriptor from phc_open()
 * @channel: Perout channel index (must be configured via phc_pin_setfunc first)
 * @start_ns: Absolute PTP time for rising edge (nanoseconds)
 * @pulse_ns: Pulse width in nanoseconds
 *
 * Returns:
 *    0: Success (pulse scheduled).
 *   -1: Failed to schedule pulse.
 */
int phc_perout_oneshot(int ptp_fd, unsigned int channel,
		       uint64_t start_ns, uint64_t pulse_ns);

/* Disable periodic output on a channel.
 * @ptp_fd: PTP device file descriptor from phc_open()
 * @channel: Perout channel index to disable
 *
 * Returns:
 *    0: Success.
 *   -1: Failed to disable.
 */
int phc_perout_disable(int ptp_fd, unsigned int channel);
