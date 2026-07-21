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
 * Generates precise square-wave clock output using BCM2711 PWM peripheral
 * with fractional clock divider for sub-ppm frequency accuracy.
 *
 * Architecture:
 * - PLLD (750 MHz base clock) → Clock Manager fractional divider →
 *   PWM peripheral (M/S mode, 1:2 ratio) → GPIO alternate function
 *
 * The Clock Manager fractional divider (MASH-1) provides the frequency
 * control. PWM acts as a pass-through to generate the square wave output.
 */

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "bcm2711-pwm-clock.h"

#define BCM2711_PERI_BASE	0xFE000000
#define GPIO_BASE		(BCM2711_PERI_BASE + 0x200000)
#define PWM_BASE		(BCM2711_PERI_BASE + 0x20C000)
#define CLOCK_BASE		(BCM2711_PERI_BASE + 0x101000)
#define BLOCK_SIZE		4096

#define GPFSEL0			0
#define GPFSEL1			1

#define PWM_CTL			0
#define PWM_STA			1
#define PWM_RNG1		4
#define PWM_DAT1		5
#define PWM_RNG2		8
#define PWM_DAT2		9

#define PCMCLK_CTL		38
#define PCMCLK_DIV		39
#define PWMCLK_CTL		40
#define PWMCLK_DIV		41

#define CLK_PASSWD		(0x5A << 24)
#define CLK_CTL_MASH(x)		((x) << 9)
#define CLK_CTL_SRC_PLLD	6
#define CLK_CTL_ENAB		(1 << 4)
#define CLK_CTL_BUSY		(1 << 7)

#define PLLD_FREQ_HZ		750000000UL

struct bcm2711_pwm_clock {
	volatile unsigned int *gpio_map;
	volatile unsigned int *pwm_map;
	volatile unsigned int *clk_map;
	uint8_t gpio_pin;
	uint8_t pwm_channel;    /* 0 or 1 (PWM0 or PWM1) */
	int mem_fd;
	double nominal_divider; /* Base frequency divider for target freq */
	int pcm_clock_enabled;  /* 1 = also write CM_PCMDIV on adjust */
	double divf_err;        /* ΔΣ error accumulator for sub-LSB resolution */
	unsigned int last_divf; /* Last written DIVF code (for diagnostics) */
};

static int get_pwm_channel(unsigned int pin, unsigned int *fsel_value)
{
	switch (pin) {
	case 12:
		*fsel_value = 4;
		return 0;
	case 18:
		*fsel_value = 2;
		return 0;
	case 13:
		*fsel_value = 4;
		return 1;
	case 19:
		*fsel_value = 2;
		return 1;
	default:
		return -1;
	}
}

int bcm2711_pwm_clock_init(uint8_t pin, struct bcm2711_pwm_clock **handle)
{
	struct bcm2711_pwm_clock *h;
	int mem_fd;
	void *gpio_map_base, *pwm_map_base, *clk_map_base;
	unsigned int fsel_value, fsel_reg, fsel_shift;
	int pwm_channel;

	pwm_channel = get_pwm_channel(pin, &fsel_value);
	if (pwm_channel < 0) {
		fprintf(stderr, "GPIO %u does not support PWM\n", pin);
		fprintf(stderr, "Supported pins: 12, 13, 18, 19\n");
		return -1;
	}

	h = calloc(1, sizeof(*h));
	if (!h) {
		fprintf(stderr, "Failed to allocate gpio_handle\n");
		return -1;
	}

	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0) {
		fprintf(stderr, "Failed to open /dev/mem (need root)\n");
		free(h);
		return -1;
	}

	gpio_map_base = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
			     MAP_SHARED, mem_fd, GPIO_BASE);
	if (gpio_map_base == MAP_FAILED) {
		fprintf(stderr, "GPIO mmap failed\n");
		close(mem_fd);
		free(h);
		return -1;
	}

	pwm_map_base = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
			    MAP_SHARED, mem_fd, PWM_BASE);
	if (pwm_map_base == MAP_FAILED) {
		fprintf(stderr, "PWM mmap failed\n");
		munmap(gpio_map_base, BLOCK_SIZE);
		close(mem_fd);
		free(h);
		return -1;
	}

	clk_map_base = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
			    MAP_SHARED, mem_fd, CLOCK_BASE);
	if (clk_map_base == MAP_FAILED) {
		fprintf(stderr, "Clock mmap failed\n");
		munmap(pwm_map_base, BLOCK_SIZE);
		munmap(gpio_map_base, BLOCK_SIZE);
		close(mem_fd);
		free(h);
		return -1;
	}

	h->gpio_map = (volatile unsigned int *)gpio_map_base;
	h->pwm_map = (volatile unsigned int *)pwm_map_base;
	h->clk_map = (volatile unsigned int *)clk_map_base;
	h->gpio_pin = pin;
	h->pwm_channel = pwm_channel;
	h->mem_fd = mem_fd;

	fsel_reg = pin / 10;
	fsel_shift = (pin % 10) * 3;
	h->gpio_map[fsel_reg] = (h->gpio_map[fsel_reg] & ~(7 << fsel_shift)) |
				(fsel_value << fsel_shift);

	fprintf(stderr, "PWM: GPIO %u configured for PWM%u\n", pin, pwm_channel);

	*handle = h;
	return 0;
}


int bcm2711_pwm_clock_prepare(struct bcm2711_pwm_clock *handle, unsigned int freq_hz)
{
	unsigned int divi, divf, range;
	unsigned long pwm_clk_target;
	double divider;
	int i;

	if (!handle)
		return -1;

	if (freq_hz >= 3000) {
		range = 512;
	} else if (freq_hz >= 750) {
		range = 2048;
	} else if (freq_hz >= 200) {
		range = 8192;
	} else {
		range = 16384;
	}

	pwm_clk_target = (unsigned long)freq_hz * range;
	divider = (double)PLLD_FREQ_HZ / pwm_clk_target;
	divi = (unsigned int)divider;
	divf = (unsigned int)((divider - divi) * 4096);

	if (divi > 4095) {
		fprintf(stderr, "Error: Divider %u exceeds 12-bit limit (freq too low)\n", divi);
		return -1;
	}

	handle->nominal_divider = divider;

	fprintf(stderr, "PWM: Target %u Hz using range=%u\n", freq_hz, range);
	fprintf(stderr, "PWM: PLLD 750MHz / %.6f = %lu Hz PWM clock\n",
		divider, pwm_clk_target);
	fprintf(stderr, "PWM: Divider DIVI=%u DIVF=%u\n", divi, divf);

	fprintf(stderr, "PWM: Disabling PWM...\n");
	handle->pwm_map[PWM_CTL] = 0;
	usleep(10);

	fprintf(stderr, "PWM: Force-killing clock...\n");
	handle->clk_map[PWMCLK_CTL] = CLK_PASSWD | (1 << 5);
	usleep(10);
	handle->clk_map[PWMCLK_CTL] = CLK_PASSWD | 0;
	usleep(100);

	fprintf(stderr, "PWM: Waiting for clock to stop (CTL=0x%x)...\n",
		handle->clk_map[PWMCLK_CTL]);

	for (i = 0; i < 100; i++) {
		if (!(handle->clk_map[PWMCLK_CTL] & CLK_CTL_BUSY))
			break;
		usleep(10);
	}

	if (i >= 100) {
		fprintf(stderr, "Warning: Clock still BUSY after %d attempts\n", i);
		fprintf(stderr, "Proceeding anyway...\n");
	} else {
		fprintf(stderr, "PWM: Clock stopped after %d attempts\n", i);
	}

	fprintf(stderr, "PWM: Setting divider...\n");
	handle->clk_map[PWMCLK_DIV] = CLK_PASSWD | (divi << 12) | divf;
	usleep(10);

	fprintf(stderr, "PWM: Starting clock (PLLD source, MASH 1-stage)...\n");
	handle->clk_map[PWMCLK_CTL] = CLK_PASSWD | CLK_CTL_ENAB | CLK_CTL_MASH(1) | CLK_CTL_SRC_PLLD;
	usleep(100);

	if (!(handle->clk_map[PWMCLK_CTL] & CLK_CTL_ENAB)) {
		fprintf(stderr, "Error: Clock failed to enable\n");
		return -1;
	}

	fprintf(stderr, "PWM: Configuring channel %u (range=%u, duty=50%%)\n",
		handle->pwm_channel, range);

	if (handle->pwm_channel == 0) {
		handle->pwm_map[PWM_RNG1] = range;
		handle->pwm_map[PWM_DAT1] = range / 2;
	} else {
		handle->pwm_map[PWM_RNG2] = range;
		handle->pwm_map[PWM_DAT2] = range / 2;
	}
	usleep(10);

	fprintf(stderr, "PWM: Prepared — output disabled, waiting for timed enable\n");
	return 0;
}

void bcm2711_pwm_clock_enable(struct bcm2711_pwm_clock *handle)
{
	if (!handle)
		return;

	if (handle->pwm_channel == 0)
		handle->pwm_map[PWM_CTL] = (1 << 7) | (1 << 0);
	else
		handle->pwm_map[PWM_CTL] = (1 << 15) | (1 << 8);
}

int bcm2711_pwm_clock_start(struct bcm2711_pwm_clock *handle, unsigned int freq_hz)
{
	int res;

	res = bcm2711_pwm_clock_prepare(handle, freq_hz);
	if (res < 0)
		return res;

	bcm2711_pwm_clock_enable(handle);
	usleep(10);

	fprintf(stderr, "PWM: Enabled\n");
	return 0;
}


int bcm2711_pwm_clock_adjust_freq(struct bcm2711_pwm_clock *handle, double ppb)
{
	double new_divider, frac, q;
	unsigned int divi, divf, div_word;

	if (!handle)
		return -1;

	/* Positive ppb = speed up clock = reduce divider */
	new_divider = handle->nominal_divider * (1.0 - ppb / 1e9);

	if (new_divider < handle->nominal_divider * 0.99 ||
	    new_divider > handle->nominal_divider * 1.01) {
		fprintf(stderr, "ERROR: PWM adjust %.1f ppb rejected! divider %.6f out of ±1%% range (nominal=%.6f)\n",
			ppb, new_divider, handle->nominal_divider);
		return -1;
	}

	divi = (unsigned int)new_divider;

	/* First-order ΔΣ: recover sub-LSB resolution by dithering DIVF */
	frac = (new_divider - (double)divi) * 4096.0;
	q = frac + handle->divf_err;
	divf = (unsigned int)lround(q);
	if (divf > 4095)
		divf = 4095;
	handle->divf_err = q - (double)divf;
	handle->last_divf = divf;

	div_word = CLK_PASSWD | (divi << 12) | divf;

	handle->clk_map[PWMCLK_DIV] = div_word;

	if (handle->pcm_clock_enabled)
		handle->clk_map[PCMCLK_DIV] = div_word;

	return 0;
}

unsigned int bcm2711_pwm_clock_get_divf(struct bcm2711_pwm_clock *handle)
{
	return handle ? handle->last_divf : 0;
}

void bcm2711_pwm_clock_enable_pcm(struct bcm2711_pwm_clock *handle)
{
	if (!handle)
		return;

	handle->pcm_clock_enabled = 1;
	fprintf(stderr, "PWM: PCM clock tracking enabled (CM_PCMDIV will follow servo)\n");
}

#define GPSET0		7
#define GPCLR0		10

int bcm2711_pwm_clock_gpio_init_output(struct bcm2711_pwm_clock *handle, uint8_t pin)
{
	unsigned int fsel_reg, fsel_shift;

	if (!handle || !handle->gpio_map || pin > 27)
		return -1;

	fsel_reg = pin / 10;
	fsel_shift = (pin % 10) * 3;

	/* Set as output (FSEL = 001) */
	handle->gpio_map[fsel_reg] = (handle->gpio_map[fsel_reg] & ~(7 << fsel_shift)) |
				     (1 << fsel_shift);

	/* Start low */
	handle->gpio_map[GPCLR0] = 1 << pin;

	return 0;
}

void bcm2711_pwm_clock_gpio_pulse(struct bcm2711_pwm_clock *handle, uint8_t pin)
{
	if (!handle || !handle->gpio_map || pin > 27)
		return;

	handle->gpio_map[GPSET0] = 1 << pin;
	for (volatile int i = 0; i < 100; i++)
		;
	handle->gpio_map[GPCLR0] = 1 << pin;
}

void bcm2711_pwm_clock_stop(struct bcm2711_pwm_clock *handle)
{
	if (!handle)
		return;

	if (handle->pwm_map)
		handle->pwm_map[PWM_CTL] = 0;

	if (handle->clk_map) {
		handle->clk_map[PWMCLK_CTL] = CLK_PASSWD | 0;
		usleep(10);
	}
}

void bcm2711_pwm_clock_cleanup(struct bcm2711_pwm_clock *handle)
{
	if (!handle)
		return;

	bcm2711_pwm_clock_stop(handle);

	if (handle->gpio_map)
		munmap((void *)handle->gpio_map, BLOCK_SIZE);

	if (handle->pwm_map)
		munmap((void *)handle->pwm_map, BLOCK_SIZE);

	if (handle->clk_map)
		munmap((void *)handle->clk_map, BLOCK_SIZE);

	if (handle->mem_fd >= 0)
		close(handle->mem_fd);

	free(handle);
}
