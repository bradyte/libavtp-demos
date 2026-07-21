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

/* Cirrus Logic CS2600 Fractional-N Clock Multiplier
 *
 * Jitter cleaner and frequency multiplier sitting between the servo's PWM
 * output and the I2S master clock:
 *
 *   BCM2711 PWM (CLK_IN) → CS2600 → CLK_OUT (MCLK) → I2S
 *                                 → AUX1_OUT (buffered CLK_IN) → i226 SDP0
 *
 * AUX1_OUT is the servo's feedback edge, so this part sits inside the
 * control loop: it both multiplies the recovered clock up to MCLK and
 * returns a buffered copy for phase measurement.
 *
 * Register access is I2C, 16-bit address and 16-bit data, MSB first
 * (datasheet figures 4-19 and 4-20).
 *
 * Configuration is applied as an ordered register sequence. Order matters:
 * mode selection and ratio must be written while the PLL is disabled, and
 * PLL_EN comes last. See cs2600_init_mult().
 */

#pragma once

#include <stdint.h>

/* Register map */
#define CS2600_REG_PLL_CFG1		0x0002
#define CS2600_REG_PLL_CFG2		0x0004
#define CS2600_REG_RATIO1_1		0x0006
#define CS2600_REG_RATIO1_2		0x0008
#define CS2600_REG_PLL_CFG3		0x0016
#define CS2600_REG_PLL_CFG4		0x001E
#define CS2600_REG_SW_RESET		0x0058
#define CS2600_REG_OUTPUT_CFG1		0x0100
#define CS2600_REG_OUTPUT_CFG2		0x0102
#define CS2600_REG_ARC_CFG1		0x0104
#define CS2600_REG_PHASE_ALIGN_CFG1	0x0108
#define CS2600_REG_DEVICE_ID1		0x0110
#define CS2600_REG_DEVICE_ID2		0x0112
#define CS2600_REG_UNLOCK_IND		0x0114
#define CS2600_REG_ERROR_STS		0x0116
#define CS2600_REG_USER_KEY		0x1104

#define CS2600_I2C_ADDR			0x2F
#define CS2600_DEVICE_ID		0x2600

#define CS2600_PLL_EN1_BIT		(1 << 8)
#define CS2600_PLL_EN2_BIT		(1 << 8)
#define CS2600_F_UNLOCK_BIT		(1 << 0)
#define CS2600_F_UNLOCK_STICKY_BIT	(1 << 1)
#define CS2600_P_UNLOCK_BIT		(1 << 2)
#define CS2600_P_UNLOCK_STICKY_BIT	(1 << 3)
#define CS2600_PHASE_TRIG_BIT		(1 << 5)

struct cs2600 {
	int fd;
	uint8_t addr;
};

struct cs2600_status {
	int freq_locked;
	int phase_locked;
	int freq_unlock_evt;	/* latched since last poll */
	int phase_unlock_evt;	/* latched since last poll */
	uint16_t err_sts;
};

/* Open the I2C bus and record the device address.
 * Returns 0 on success, -1 on error. */
int cs2600_open(const char *i2c_dev_path, uint8_t addr7, struct cs2600 *dev);

void cs2600_close(struct cs2600 *dev);

/* Read DEVICE_ID1 and check it reads 0x2600.
 * Returns 0 if present, -1 otherwise. */
int cs2600_check_id(struct cs2600 *dev);

/* Configure Multiplier Mode: reset, program the ratio and output dividers,
 * enable the PLL, then wait for frequency lock.
 *
 * @mclk_hz:   desired CLK_OUT, the I2S master clock
 * @clk_in_hz: frequency present on CLK_IN, the servo's PWM output
 *
 * The multiplication ratio is mclk_hz / clk_in_hz, encoded Q20.12, so
 * 24.576 MHz from 300 Hz gives 81920.0. Nothing here hardcodes either rate;
 * the caller derives clk_in_hz from the CRF stream profile.
 *
 * Reference clock is the on-board 12 MHz crystal on REF_CLK_IN, FLL
 * bandwidth 1 Hz for maximum jitter rejection, BCLK = CLK_OUT/8,
 * FSYNC = CLK_OUT/512, AUX1_OUT = buffered CLK_IN for the servo feedback
 * path, automatic phase alignment, holdover enabled.
 *
 * Returns:
 *    0: locked.
 *   -1: I2C error, or a ratio outside what Q20.12 can represent.
 *   -2: frequency lock timeout. */
int cs2600_init_mult(struct cs2600 *dev, double mclk_hz, double clk_in_hz);

/* Poll lock state and error flags. Sticky bits are cleared as a side effect,
 * so each call reports events since the previous one.
 * Returns 0 on success, -1 on I2C error. */
int cs2600_get_status(struct cs2600 *dev, struct cs2600_status *st);

/* Change FLL bandwidth. FLL_BW is not in the live-writable set, so the PLL
 * is disabled around the write and restored afterwards; the output gates
 * cleanly during this because OUT_GATE_TYPE is F_UNLOCK.
 * Returns 0 on success, -1 on I2C error. */
int cs2600_set_fll_bw(struct cs2600 *dev, uint8_t bw_sel, int mult16);

/* Request a manual phase alignment. Only meaningful when phase alignment is
 * configured for manual triggering.
 * Returns 0 on success, -1 on I2C error. */
int cs2600_trigger_phase_align(struct cs2600 *dev);
