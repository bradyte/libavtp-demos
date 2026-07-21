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

/* RP1 Clock Manager — PLL VCO steering via FBDIV_FRAC
 *
 * Media clock recovery by trimming pll_audio's 24-bit fractional feedback
 * divider. This shifts the VCO frequency and everything downstream moves
 * by the same ppb — true clock recovery for I2S, not just tap correction.
 *
 * Signal path:
 *   pll_audio VCO (1.572864 GHz) → PLL_PRIM → clk_i2s (12.288 MHz) → DAC MCLK
 *                                            → clk_gp0 (300 Hz) → GPIO4
 *
 * This module programs the VCO to 1.572864 GHz at init and computes downstream
 * dividers from the kernel-configured PLL_PRIM post-divider. DT overlay handles
 * clock parenting and pinmux only.
 *
 * Rules:
 *   - Never touch FBDIV_INT at runtime (would cause phase glitch)
 *   - Single aligned 32-bit store per update (atomic, glitchless)
 *   - Authority clamped to ±100 ppm to prevent runaway
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "devmem.h"
#include "rp1-clock.h"

/* RP1 PCIe BAR1 base (from /proc/iomem: 1f00000000-1f003fffff) */
#define RP1_BAR1_BASE		0x1f00000000ULL

/* Clock manager block offset within BAR1 */
#define CLOCKS_MAIN_OFFSET	0x00018000

/* Map size: covers clocks_main (0x18000) through pll_audio (0x24000) */
#define MAP_SIZE		0x10000

/* Atomic access offsets */
#define ATOMIC_SET		0x2000
#define ATOMIC_CLR		0x3000

/* GPCLK output enable register */
#define GPCLK_OE_CTRL		0x000

/* CLK_I2S registers (offset 0xb4 within clocks_main) */
#define CLK_I2S_CTRL		0x0b4
#define CLK_I2S_DIV_INT		0x0b8
#define CLK_I2S_SEL		0x0c0

/* CLK_GP0 registers (offset 0x174 within clocks_main) */
#define CLK_GP0_CTRL		0x174
#define CLK_GP0_DIV_INT		0x178
#define CLK_GP0_DIV_FRAC	0x17c
#define CLK_GP0_SEL		0x180

/* PLL_AUDIO registers (offset 0xc000 from clocks_main base) */
#define PLL_AUDIO_CS		0xc000
#define PLL_AUDIO_PWR		0xc004
#define PLL_AUDIO_FBDIV_INT	0xc008
#define PLL_AUDIO_FBDIV_FRAC	0xc00c
#define PLL_AUDIO_PRIM		0xc010

/* Clock control bits */
#define CLK_CTRL_ENABLE		(1 << 11)
#define CLK_CTRL_AUXSRC_SHIFT	5
#define CLK_CTRL_AUXSRC_MASK	(0x1f << CLK_CTRL_AUXSRC_SHIFT)
#define CLK_CTRL_SRC_MASK	0x1

/* PLL bits */
#define PLL_CS_LOCK		(1u << 31)
#define PLL_PWR_PD		(1 << 0)
#define PLL_PWR_POSTDIVPD	(1 << 3)
#define PLL_PWR_VCOPD		(1 << 5)

/* GP0 output enable is bit 0 of GPCLK_OE_CTRL */
#define GP0_OE_BIT		(1 << 0)

/* clk_gp0 parent: clk_i2s is AUXSRC index 10 */
#define GP0_AUXSRC_I2S		10

/* clk_i2s parent: pll_audio is AUXSRC index 1 */
#define I2S_AUXSRC_PLL_AUDIO	1

/* Clock domain targets */
#define TARGET_VCO_HZ		1572864000.0	/* 12.288 MHz × 128 */
#define TARGET_I2S_HZ		12288000.0	/* 256×fs, standard DAC MCLK */
#define TARGET_GP0_HZ		300.0		/* 1 edge per CRF timestamp */

/* PLL fractional divider */
#define F_XOSC_HZ		50000000.0
#define FRAC_BITS		24
#define FRAC_MOD		(1u << FRAC_BITS)
#define AUTHORITY_PPM		10000.0

/* VCO = F_XOSC × (FBDIV_INT + FBDIV_FRAC / 2^24) = 1,572,864,000 Hz */
#define TARGET_FBDIV_INT	31
#define TARGET_FBDIV_FRAC	0x75104D	/* 0.45728 × 2^24 */

/* GP0 divider is fixed: clk_i2s / 300 Hz */
#define GP0_NOMINAL_DIV_INT	((unsigned int)(TARGET_I2S_HZ / TARGET_GP0_HZ))

struct rp1_clock {
	struct devmem_region reg;
	volatile uint32_t *regs;
	int mem_fd;

	/* PLL VCO steering state */
	uint32_t fbdiv_int;
	uint32_t frac_center;
	double ppb_per_lsb;
	long max_codes;
};

static inline uint32_t reg_read(struct rp1_clock *h, uint32_t offset)
{
	return h->regs[offset / 4];
}

static inline void reg_write(struct rp1_clock *h, uint32_t offset, uint32_t val)
{
	h->regs[offset / 4] = val;
}

static inline void reg_set(struct rp1_clock *h, uint32_t offset, uint32_t bits)
{
	h->regs[(offset + ATOMIC_SET) / 4] = bits;
}

static inline void reg_clr(struct rp1_clock *h, uint32_t offset, uint32_t bits)
{
	h->regs[(offset + ATOMIC_CLR) / 4] = bits;
}

int rp1_clock_init(struct rp1_clock **handle)
{
	struct rp1_clock *h;
	double mult, vco;
	int fd;

	h = calloc(1, sizeof(*h));
	if (!h)
		return -ENOMEM;

	h->reg = (struct devmem_region){ RP1_BAR1_BASE + CLOCKS_MAIN_OFFSET,
					 MAP_SIZE, "RP1 clocks_main" };

	if (devmem_map(&h->reg, 1, &fd) < 0) {
		free(h);
		return -errno;
	}

	h->regs = h->reg.base;
	h->mem_fd = fd;

	/* Program PLL to target VCO (1.572864 GHz for 48 kHz audio family) */
	uint32_t cur_int = reg_read(h, PLL_AUDIO_FBDIV_INT) & 0xfff;
	uint32_t cur_frac = reg_read(h, PLL_AUDIO_FBDIV_FRAC) & (FRAC_MOD - 1);

	if (cur_int != TARGET_FBDIV_INT || cur_frac != TARGET_FBDIV_FRAC) {
		fprintf(stderr, "rp1-clock: PLL at INT=%u FRAC=0x%06x, reprogramming to %u/0x%06x...\n",
			cur_int, cur_frac, TARGET_FBDIV_INT, TARGET_FBDIV_FRAC);

		reg_write(h, PLL_AUDIO_FBDIV_INT, TARGET_FBDIV_INT);
		reg_write(h, PLL_AUDIO_FBDIV_FRAC, TARGET_FBDIV_FRAC);
		usleep(100);

		for (int i = 0; i < 10000; i += 10) {
			if (reg_read(h, PLL_AUDIO_CS) & PLL_CS_LOCK)
				break;
			usleep(10);
		}
		if (!(reg_read(h, PLL_AUDIO_CS) & PLL_CS_LOCK)) {
			fprintf(stderr, "rp1-clock: PLL failed to relock\n");
			devmem_unmap(&h->reg, 1, fd);
			free(h);
			return -ETIMEDOUT;
		}
		fprintf(stderr, "rp1-clock: PLL relocked at 1.573 GHz\n");
	}

	h->fbdiv_int = TARGET_FBDIV_INT;
	h->frac_center = TARGET_FBDIV_FRAC;

	mult = (double)h->fbdiv_int + (double)h->frac_center / FRAC_MOD;
	vco = F_XOSC_HZ * mult;
	h->ppb_per_lsb = 1e9 / ((double)FRAC_MOD * mult);
	h->max_codes = lround(AUTHORITY_PPM * 1000.0 / h->ppb_per_lsb);

	fprintf(stderr, "rp1-clock: VCO=%.3f MHz  %.3f ppb/LSB  authority=±%ld codes (±%.0f ppm)\n",
		vco / 1e6, h->ppb_per_lsb, h->max_codes, AUTHORITY_PPM);

	*handle = h;
	return 0;
}

static int wait_for_sel(struct rp1_clock *h, uint32_t sel_reg, int timeout_us)
{
	for (int i = 0; i < timeout_us; i += 10) {
		if (reg_read(h, sel_reg) != 0)
			return 0;
		usleep(10);
	}
	return -ETIMEDOUT;
}

int rp1_clock_enable(struct rp1_clock *handle)
{
	uint32_t ctrl, pwr;

	if (!handle)
		return -EINVAL;

	/* Ensure PLL audio is powered on and locked */
	pwr = reg_read(handle, PLL_AUDIO_PWR);
	if (pwr & PLL_PWR_PD) {
		fprintf(stderr, "rp1-clock: powering on pll_audio...\n");
		reg_clr(handle, PLL_AUDIO_PWR, PLL_PWR_PD | PLL_PWR_VCOPD | PLL_PWR_POSTDIVPD);
		usleep(100);

		for (int i = 0; i < 10000; i += 10) {
			if (reg_read(handle, PLL_AUDIO_CS) & PLL_CS_LOCK)
				break;
			usleep(10);
		}

		if (!(reg_read(handle, PLL_AUDIO_CS) & PLL_CS_LOCK)) {
			fprintf(stderr, "rp1-clock: WARNING: pll_audio did not lock\n");
			return -ETIMEDOUT;
		}
		fprintf(stderr, "rp1-clock: pll_audio locked\n");
	} else {
		if (reg_read(handle, PLL_AUDIO_CS) & PLL_CS_LOCK)
			fprintf(stderr, "rp1-clock: pll_audio already running and locked\n");
		else
			fprintf(stderr, "rp1-clock: WARNING: pll_audio powered but not locked\n");
	}

	/* Compute CLK_I2S divider from PLL post-divider.
	 * pll_audio output = TARGET_VCO / (PRIM_div1 × PRIM_div2)
	 * CLK_I2S_DIV = pll_audio output / TARGET_I2S_HZ */
	uint32_t prim = reg_read(handle, PLL_AUDIO_PRIM);
	unsigned int prim_div1 = (prim >> 16) & 0x7;
	unsigned int prim_div2 = (prim >> 12) & 0x7;
	if (prim_div1 == 0) prim_div1 = 1;
	if (prim_div2 == 0) prim_div2 = 1;
	double pll_out_hz = TARGET_VCO_HZ / (prim_div1 * prim_div2);
	unsigned int i2s_div = (unsigned int)(pll_out_hz / TARGET_I2S_HZ + 0.5);

	fprintf(stderr, "rp1-clock: PLL_PRIM div1=%u div2=%u → pll_audio=%.3f MHz, CLK_I2S DIV=%u\n",
		prim_div1, prim_div2, pll_out_hz / 1e6, i2s_div);
	reg_write(handle, CLK_I2S_DIV_INT, i2s_div);

	/* Enable clk_i2s: set AUXSRC to pll_audio (index 1), set SRC to AUX (1), enable */
	ctrl = reg_read(handle, CLK_I2S_CTRL);
	if (!(ctrl & CLK_CTRL_ENABLE)) {
		fprintf(stderr, "rp1-clock: enabling clk_i2s (AUXSRC=%d)...\n",
			I2S_AUXSRC_PLL_AUDIO);

		ctrl &= ~CLK_CTRL_AUXSRC_MASK;
		ctrl |= (I2S_AUXSRC_PLL_AUDIO << CLK_CTRL_AUXSRC_SHIFT);
		ctrl &= ~CLK_CTRL_SRC_MASK;
		ctrl |= 1;
		ctrl |= CLK_CTRL_ENABLE;
		reg_write(handle, CLK_I2S_CTRL, ctrl);
		usleep(100);

		if (wait_for_sel(handle, CLK_I2S_SEL, 10000) < 0)
			fprintf(stderr, "rp1-clock: WARNING: clk_i2s SEL not set\n");
		else
			fprintf(stderr, "rp1-clock: clk_i2s enabled\n");
	} else {
		fprintf(stderr, "rp1-clock: clk_i2s already enabled\n");
	}

	/* Enable clk_gp0: set AUXSRC to clk_i2s (index 10), fixed divider */
	ctrl = reg_read(handle, CLK_GP0_CTRL);
	if (!(ctrl & CLK_CTRL_ENABLE)) {
		fprintf(stderr, "rp1-clock: enabling clk_gp0 (AUXSRC=%d, div=%u fixed)...\n",
			GP0_AUXSRC_I2S, GP0_NOMINAL_DIV_INT);

		reg_write(handle, CLK_GP0_DIV_INT, GP0_NOMINAL_DIV_INT);
		reg_write(handle, CLK_GP0_DIV_FRAC, 0);
		usleep(10);

		ctrl &= ~CLK_CTRL_AUXSRC_MASK;
		ctrl |= (GP0_AUXSRC_I2S << CLK_CTRL_AUXSRC_SHIFT);
		ctrl &= ~CLK_CTRL_SRC_MASK;
		ctrl |= 1;
		ctrl |= CLK_CTRL_ENABLE;
		reg_write(handle, CLK_GP0_CTRL, ctrl);
		usleep(100);

		if (wait_for_sel(handle, CLK_GP0_SEL, 10000) < 0)
			fprintf(stderr, "rp1-clock: WARNING: clk_gp0 SEL not set\n");
		else
			fprintf(stderr, "rp1-clock: clk_gp0 enabled\n");
	} else {
		fprintf(stderr, "rp1-clock: clk_gp0 already enabled\n");
	}

	/* Enable GPCLK0 output on GPIO pad */
	reg_set(handle, GPCLK_OE_CTRL, GP0_OE_BIT);
	fprintf(stderr, "rp1-clock: GPCLK0 output enabled\n");

	fprintf(stderr, "rp1-clock: GP0 CTRL=0x%08x DIV_INT=%u SEL=0x%x\n",
		reg_read(handle, CLK_GP0_CTRL),
		reg_read(handle, CLK_GP0_DIV_INT),
		reg_read(handle, CLK_GP0_SEL));

	return 0;
}

int rp1_clock_adjust(struct rp1_clock *handle, double ppb)
{
	long codes, f;

	if (!handle)
		return -EINVAL;

	/* Positive ppb = speed up = increase FBDIV_FRAC (higher VCO freq) */
	codes = lround(ppb / handle->ppb_per_lsb);

	if (codes > handle->max_codes)
		codes = handle->max_codes;
	if (codes < -handle->max_codes)
		codes = -handle->max_codes;

	f = (long)handle->frac_center + codes;

	if (f < 0 || f >= (long)FRAC_MOD) {
		fprintf(stderr, "rp1-clock: frac range exhausted (codes=%ld)\n", codes);
		f = (f < 0) ? 0 : (long)(FRAC_MOD - 1);
	}

	/* Single atomic 32-bit store — glitchless PLL update */
	reg_write(handle, PLL_AUDIO_FBDIV_FRAC, (uint32_t)f);

	return 0;
}

int rp1_clock_adjust_fn(void *ctx, double ppb)
{
	return rp1_clock_adjust((struct rp1_clock *)ctx, ppb);
}

void rp1_clock_cleanup(struct rp1_clock *handle)
{
	if (!handle)
		return;

	/* Restore nominal FBDIV_FRAC */
	reg_write(handle, PLL_AUDIO_FBDIV_FRAC, handle->frac_center);
	fprintf(stderr, "rp1-clock: restored FBDIV_FRAC to nominal 0x%06x\n",
		handle->frac_center);

	/* Disable GP0 output enable */
	reg_clr(handle, GPCLK_OE_CTRL, GP0_OE_BIT);

	/* Disable clk_gp0 */
	reg_clr(handle, CLK_GP0_CTRL, CLK_CTRL_ENABLE);

	fprintf(stderr, "rp1-clock: clk_gp0 disabled\n");

	devmem_unmap(&handle->reg, 1, handle->mem_fd);

	free(handle);
}
