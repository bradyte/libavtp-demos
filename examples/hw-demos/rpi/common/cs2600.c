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

/* Cirrus Logic CS2600 Fractional-N Clock Multiplier.
 *
 * Multiplier Mode configuration validated on CDB2600-DC-SD via I2C at 0x2F,
 * 2026-07-07.
 */

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "cs2600.h"

/* Q20.12 fixed point: 20 integer bits, 12 fractional. */
#define CS2600_RATIO_FRAC_BITS		12
#define CS2600_RATIO_MAX		(1u << 20)

/* Frequency lock takes up to ~700 CLK_IN periods. Allow a wide margin: at
 * 300 Hz that is ~2.3 s, and this yields ~3 s. */
#define CS2600_LOCK_UI			700
#define CS2600_LOCK_MARGIN		1.3
#define CS2600_LOCK_POLL_US		1000

/* One entry of an ordered configuration sequence. */
struct cs2600_reg {
	uint16_t reg;
	uint16_t val;
	const char *what;
};

/* 16-bit register address followed by 16-bit data, MSB first (fig 4-19). */
static int cs2600_write_reg(struct cs2600 *dev, uint16_t reg, uint16_t val)
{
	uint8_t buf[4] = { reg >> 8, reg & 0xFF, val >> 8, val & 0xFF };
	struct i2c_msg m = { dev->addr, 0, sizeof(buf), buf };
	struct i2c_rdwr_ioctl_data x = { &m, 1 };

	if (ioctl(dev->fd, I2C_RDWR, &x) < 0) {
		perror("cs2600_write_reg");
		return -1;
	}

	return 0;
}

/* Address write then data read as one combined transfer, so the bus is not
 * released between them (repeated start, fig 4-20). */
static int cs2600_read_reg(struct cs2600 *dev, uint16_t reg, uint16_t *val)
{
	uint8_t a[2] = { reg >> 8, reg & 0xFF };
	uint8_t d[2] = { 0, 0 };
	struct i2c_msg m[2] = {
		{ dev->addr, 0,		sizeof(a), a },
		{ dev->addr, I2C_M_RD,	sizeof(d), d },
	};
	struct i2c_rdwr_ioctl_data x = { m, 2 };

	if (ioctl(dev->fd, I2C_RDWR, &x) < 0) {
		perror("cs2600_read_reg");
		return -1;
	}

	*val = ((uint16_t)d[0] << 8) | d[1];
	return 0;
}

static int cs2600_modify_reg(struct cs2600 *dev, uint16_t reg, uint16_t mask,
			     uint16_t val)
{
	uint16_t cur;

	if (cs2600_read_reg(dev, reg, &cur) < 0)
		return -1;

	return cs2600_write_reg(dev, reg, (cur & ~mask) | (val & mask));
}

/* Apply an ordered register sequence, stopping at the first failure.
 *
 * The previous implementation ignored every write result after the first,
 * so a bus glitch left the PLL half configured while init reported success —
 * in a part that sits inside the servo loop. */
static int cs2600_apply(struct cs2600 *dev, const struct cs2600_reg *seq,
			size_t count)
{
	size_t i;

	for (i = 0; i < count; i++) {
		if (cs2600_write_reg(dev, seq[i].reg, seq[i].val) < 0) {
			fprintf(stderr,
				"cs2600: failed writing 0x%04x = 0x%04x (%s)\n",
				seq[i].reg, seq[i].val, seq[i].what);
			return -1;
		}
	}

	return 0;
}

int cs2600_open(const char *i2c_dev_path, uint8_t addr7, struct cs2600 *dev)
{
	if (!i2c_dev_path || !dev)
		return -1;

	dev->addr = addr7;
	dev->fd = open(i2c_dev_path, O_RDWR);
	if (dev->fd < 0) {
		fprintf(stderr, "cs2600: failed to open %s\n", i2c_dev_path);
		return -1;
	}

	return 0;
}

void cs2600_close(struct cs2600 *dev)
{
	if (dev && dev->fd >= 0) {
		close(dev->fd);
		dev->fd = -1;
	}
}

int cs2600_check_id(struct cs2600 *dev)
{
	uint16_t id;

	if (cs2600_read_reg(dev, CS2600_REG_DEVICE_ID1, &id) < 0)
		return -1;

	return id == CS2600_DEVICE_ID ? 0 : -1;
}

static int cs2600_reset(struct cs2600 *dev)
{
	if (cs2600_write_reg(dev, CS2600_REG_SW_RESET, 0x005A) < 0)
		return -1;

	usleep(100000);

	return cs2600_check_id(dev);
}

/* Wait for the frequency unlock indicator to clear, then clear the sticky
 * bits so later polls report only new events. */
static int cs2600_wait_lock(struct cs2600 *dev, unsigned int timeout_ms)
{
	unsigned int i;

	for (i = 0; i < timeout_ms; i++) {
		uint16_t u;

		if (cs2600_read_reg(dev, CS2600_REG_UNLOCK_IND, &u) < 0)
			return -1;

		if (!(u & CS2600_F_UNLOCK_BIT)) {
			cs2600_write_reg(dev, CS2600_REG_UNLOCK_IND,
					 CS2600_F_UNLOCK_STICKY_BIT |
					 CS2600_P_UNLOCK_STICKY_BIT);
			return 0;
		}

		usleep(CS2600_LOCK_POLL_US);
	}

	return -2;
}

int cs2600_init_mult(struct cs2600 *dev, double mclk_hz, double clk_in_hz)
{
	unsigned int timeout_ms;
	uint32_t ratio_q20_12;
	double ratio;

	if (!dev || clk_in_hz <= 0.0 || mclk_hz <= 0.0)
		return -1;

	ratio = mclk_hz / clk_in_hz;
	if (ratio >= (double)CS2600_RATIO_MAX) {
		fprintf(stderr, "cs2600: ratio %.3f exceeds Q20.12 range\n",
			ratio);
		return -1;
	}

	ratio_q20_12 = (uint32_t)(ratio * (1u << CS2600_RATIO_FRAC_BITS) + 0.5);

	/* Reset to a known state before touching configuration. */
	if (cs2600_reset(dev) < 0)
		return -1;

	/* Order is significant: mode and ratio are programmed with the PLL
	 * disabled, and PLL_EN1/PLL_EN2 come last. */
	const struct cs2600_reg seq[] = {
		{ CS2600_REG_PLL_CFG2, 0x0009,
		  "PLL_MODE_SEL = Multiplier, M_RATIO_SEL = Ratio1" },
		{ CS2600_REG_RATIO1_1, ratio_q20_12 >> 16,
		  "multiplication ratio, high half" },
		{ CS2600_REG_RATIO1_2, ratio_q20_12 & 0xFFFF,
		  "multiplication ratio, low half" },
		{ CS2600_REG_PLL_CFG3, 0x0012,
		  "REF_CLK_IN divide by 1, SYSCLK_SRC = REF_CLK_IN" },
		{ CS2600_REG_PLL_CFG4, 0x0000,
		  "FLL bandwidth 1 Hz, maximum jitter rejection" },
		{ CS2600_REG_OUTPUT_CFG1, 0x5582,
		  "BCLK = CLK_OUT/8, FSYNC = CLK_OUT/512, both inverted" },
		{ CS2600_REG_OUTPUT_CFG2, 0x0400,
		  "AUX1_OUT = buffered CLK_IN, CLK_OUT = MCLK" },
		{ CS2600_REG_PHASE_ALIGN_CFG1, 0x8008,
		  "phase alignment automatic, threshold 8 MCLK" },
		{ CS2600_REG_PLL_CFG1, 0x0980,
		  "PLL_EN1 = 1, S_RATIO_SEL = Ratio2 for holdover" },
		{ CS2600_REG_PLL_CFG2, 0x0109,
		  "PLL_EN2 = 1, PLL_MODE_SEL = Multiplier" },
	};

	if (cs2600_apply(dev, seq, sizeof(seq) / sizeof(seq[0])) < 0)
		return -1;

	timeout_ms = (unsigned int)(CS2600_LOCK_UI * 1000.0 * CS2600_LOCK_MARGIN
				    / clk_in_hz);

	return cs2600_wait_lock(dev, timeout_ms);
}

int cs2600_get_status(struct cs2600 *dev, struct cs2600_status *st)
{
	uint16_t u, e;

	if (!dev || !st)
		return -1;

	if (cs2600_read_reg(dev, CS2600_REG_UNLOCK_IND, &u) < 0)
		return -1;
	if (cs2600_read_reg(dev, CS2600_REG_ERROR_STS, &e) < 0)
		return -1;

	st->freq_locked      = !(u & CS2600_F_UNLOCK_BIT);
	st->phase_locked     = !(u & CS2600_P_UNLOCK_BIT);
	st->freq_unlock_evt  = !!(u & CS2600_F_UNLOCK_STICKY_BIT);
	st->phase_unlock_evt = !!(u & CS2600_P_UNLOCK_STICKY_BIT);
	st->err_sts = e;

	/* Clear latched state so the next poll reports only new events. */
	if (cs2600_write_reg(dev, CS2600_REG_UNLOCK_IND,
			     CS2600_F_UNLOCK_STICKY_BIT |
			     CS2600_P_UNLOCK_STICKY_BIT) < 0)
		return -1;

	if (e && cs2600_write_reg(dev, CS2600_REG_ERROR_STS, e) < 0)
		return -1;

	return 0;
}

int cs2600_set_fll_bw(struct cs2600 *dev, uint8_t bw_sel, int mult16)
{
	uint16_t c1, c2;

	if (!dev)
		return -1;

	if (cs2600_read_reg(dev, CS2600_REG_PLL_CFG1, &c1) < 0)
		return -1;
	if (cs2600_read_reg(dev, CS2600_REG_PLL_CFG2, &c2) < 0)
		return -1;

	const struct cs2600_reg seq[] = {
		{ CS2600_REG_PLL_CFG1, c1 & ~CS2600_PLL_EN1_BIT,
		  "disable PLL_EN1 around FLL_BW write" },
		{ CS2600_REG_PLL_CFG2, c2 & ~CS2600_PLL_EN2_BIT,
		  "disable PLL_EN2 around FLL_BW write" },
		{ CS2600_REG_PLL_CFG4,
		  (uint16_t)(((mult16 ? 1 : 0) << 7) | ((bw_sel & 7) << 4)),
		  "FLL bandwidth" },
		{ CS2600_REG_PLL_CFG1, c1, "restore PLL_CFG1" },
		{ CS2600_REG_PLL_CFG2, c2, "restore PLL_CFG2" },
	};

	return cs2600_apply(dev, seq, sizeof(seq) / sizeof(seq[0]));
}

int cs2600_trigger_phase_align(struct cs2600 *dev)
{
	if (!dev)
		return -1;

	return cs2600_modify_reg(dev, CS2600_REG_PHASE_ALIGN_CFG1,
				 CS2600_PHASE_TRIG_BIT, CS2600_PHASE_TRIG_BIT);
}
