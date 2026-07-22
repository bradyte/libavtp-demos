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

/* PTP external timestamp capture and pin multiplexing.
 *
 * Thin wrappers over the Linux PTP ioctls that hardware-timestamp an input
 * edge: PTP_PIN_SETFUNC to route a physical pin to a capture channel, and
 * PTP_EXTTS_REQUEST to arm it. Events are then read from the PHC file
 * descriptor as struct ptp_extts_event.
 *
 * These calls carry no NIC-specific knowledge - they take whatever pin and
 * channel index the caller supplies. The hardware knowledge is the caller's:
 *
 *   which physical pin is which index   i210: 0=SDP0 1=SDP1 2=U.FL1 3=U.FL2
 *                                       i226: different mapping
 *   which channels exist                varies, and many NICs have none
 *   whether edge selection is honoured   i210 captures both edges in
 *                                        hardware regardless of the request
 *
 * Separated from phc-utils.h because that half works on any PHC-capable NIC,
 * while nothing here is guaranteed to exist at all. A platform without
 * external timestamp capture links phc-utils and not this.
 */

#pragma once

#include <stdint.h>

/* Route a physical pin to a capture channel.
 *
 * @pin:  hardware pin index, NIC-specific
 * @func: 0 none, 1 extts, 2 perout, 3 pps
 * @chan: channel index to associate with the pin
 *
 * Example, i226 SDP0 to capture channel 0:
 *   ptp_pin_setfunc(fd, 0, 1, 0);
 *
 * Returns 0 on success, -1 on error. Many NICs have fixed-function pins and
 * fail this. */
int ptp_pin_setfunc(int ptp_fd, unsigned int pin, unsigned int func,
		    unsigned int chan);

/* Arm external timestamp capture on a channel.
 * @rising: 1 for rising edges, 0 for falling.
 * Returns 0 on success, -1 on error. */
int ptp_extts_enable(int ptp_fd, unsigned int index, int rising);

/* Disarm external timestamp capture on a channel.
 * Returns 0 on success, -1 on error. */
int ptp_extts_disable(int ptp_fd, unsigned int index);

/* Read the next event for @channel, blocking. Events belonging to other
 * channels are skipped rather than returned or discarded silently.
 * Returns 0 on success, -1 on error. */
int ptp_extts_read(int ptp_fd, unsigned int channel, uint64_t *timestamp_ns);

/* As above with a bound on how long to wait.
 *
 * @timeout_ms: 0 returns immediately if nothing is queued, a positive value
 *              waits at most that long, negative blocks like the above.
 *
 * Returns 0 on success, -1 if nothing arrived in time or on error. */
int ptp_extts_read_timeout(int ptp_fd, unsigned int channel,
			   uint64_t *timestamp_ns, int timeout_ms);
