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

/* BCM2711 PWM Clock Generator HAL
 *
 * Hardware: BCM2711 (Raspberry Pi 4, CM4)
 * Peripherals: PWM0/1, Clock Manager GP0/1/2, GPIO alternate functions
 *
 * Generates precise square-wave clock output using hardware PWM with
 * fractional divider frequency control for sub-ppm accuracy.
 *
 * Supported GPIO pins: 12, 13, 18, 19 (PWM-capable pins only)
 * Frequency range: ~1 Hz to 25 MHz (limited by PLLD fractional divider)
 * Frequency adjustment: ±500 ppm via PPB (parts-per-billion)
 *
 * Requirements:
 * - Root access (direct /dev/mem mapping)
 * - BCM2711 hardware (will fail on other SoCs)
 */

#pragma once

#include <stdint.h>

struct bcm2711_pwm_clock;

/* Initialize PWM clock on specified GPIO pin (single-channel mode)
 * Returns 0 on success, negative on error */
int bcm2711_pwm_clock_init(uint8_t gpio_pin, struct bcm2711_pwm_clock **handle);

/* Start clock generation at specified frequency (Hz) - single channel
 * Returns 0 on success, negative on error */
int bcm2711_pwm_clock_start(struct bcm2711_pwm_clock *handle, uint32_t freq_hz);

/* Prepare clock hardware without enabling PWM output.
 * Configures clock divider, PWM range/dat, but leaves PWM_CTL disabled.
 * Call bcm2711_pwm_clock_enable() to start output at a precise moment.
 * Returns 0 on success, negative on error */
int bcm2711_pwm_clock_prepare(struct bcm2711_pwm_clock *handle, uint32_t freq_hz);

/* Enable PWM output (single register write).
 * Must call bcm2711_pwm_clock_prepare() first.
 * Designed for timed enable: sleep until target time, then call this. */
void bcm2711_pwm_clock_enable(struct bcm2711_pwm_clock *handle);

/* Adjust frequency by PPB (parts-per-billion)
 * ppb: -500000.0 to +500000.0 (±500 ppm)
 * Writes CM_PWMDIV (always) and CM_PCMDIV (if PCM clock enabled).
 * Returns 0 on success, negative on error */
int bcm2711_pwm_clock_adjust_freq(struct bcm2711_pwm_clock *handle, double ppb);

/* Enable PCM clock tracking.
 * When enabled, bcm2711_pwm_clock_adjust_freq() also writes CM_PCMDIV
 * so the I2S peripheral tracks the same recovered media clock.
 * Call after DT overlay configures I2S master mode on GPIO 18-21. */
void bcm2711_pwm_clock_enable_pcm(struct bcm2711_pwm_clock *handle);

/* Get last written DIVF code (for diagnostics) */
unsigned int bcm2711_pwm_clock_get_divf(struct bcm2711_pwm_clock *handle);

/* Configure a GPIO pin as output and set it low.
 * Used for FPGA divider sync reset. Pin must not be a PWM pin (12/13/18/19). */
int bcm2711_pwm_clock_gpio_init_output(struct bcm2711_pwm_clock *handle, uint8_t pin);

/* Pulse a GPIO pin high for ~1µs then low. */
void bcm2711_pwm_clock_gpio_pulse(struct bcm2711_pwm_clock *handle, uint8_t pin);

/* Stop clock generation (keeps hardware initialized) */
void bcm2711_pwm_clock_stop(struct bcm2711_pwm_clock *handle);

/* Cleanup and release hardware resources */
void bcm2711_pwm_clock_cleanup(struct bcm2711_pwm_clock *handle);
