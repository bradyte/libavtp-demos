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

/* RP1 Clock Manager HAL — Raspberry Pi 5
 *
 * Programs pll_audio VCO to 1.572864 GHz and steers FBDIV_FRAC for
 * glitchless media clock recovery. DT overlay handles parenting/pinmux.
 *
 * Requirements:
 *   - Root access (/dev/mem)
 *   - DT overlay loaded: dtoverlay=crf-clk-tap
 *   - Raspberry Pi 5 hardware (RP1 at PCIe BAR1)
 */

#pragma once

#include <stdint.h>

struct rp1_clock;

/* Initialize RP1 clock manager mmap.
 * Programs pll_audio VCO to 1.572864 GHz if not already set.
 * Returns 0 on success, negative on error. */
int rp1_clock_init(struct rp1_clock **handle);

/* Enable clk_i2s and clk_gp0. Assumes DT has set parents and dividers.
 * Returns 0 on success, negative on error. */
int rp1_clock_enable(struct rp1_clock *handle);

/* Adjust pll_audio VCO frequency by ppb (parts-per-billion).
 * Positive ppb = speed up clock. Clamped to ±100 ppm authority.
 * Returns 0 on success, negative on error. */
int rp1_clock_adjust(struct rp1_clock *handle, double ppb);

/* clock_adjust_fn compatible wrapper for use with PI servo */
int rp1_clock_adjust_fn(void *ctx, double ppb);

/* Stop clocks and release resources */
void rp1_clock_cleanup(struct rp1_clock *handle);
