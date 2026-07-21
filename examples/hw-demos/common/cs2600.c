#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "cs2600.h"

int cs2600_open(const char *path, uint8_t addr7, struct cs2600 *dev) {
    dev->fd = open(path, O_RDWR);
    dev->addr = addr7;
    return dev->fd < 0 ? -1 : 0;
}
void cs2600_close(struct cs2600 *dev) { if (dev->fd >= 0) close(dev->fd); }

/* 16-bit register address + 16-bit data word, MSB-first (Fig 4-19/4-20) */
int cs2600_write_reg(struct cs2600 *dev, uint16_t reg, uint16_t val) {
    uint8_t buf[4] = { reg >> 8, reg & 0xFF, val >> 8, val & 0xFF };
    struct i2c_msg m = { dev->addr, 0, sizeof(buf), buf };
    struct i2c_rdwr_ioctl_data x = { &m, 1 };
    if (ioctl(dev->fd, I2C_RDWR, &x) < 0) { perror("cs2600_write_reg"); return -1; }
    return 0;
}

int cs2600_read_reg(struct cs2600 *dev, uint16_t reg, uint16_t *val) {
    uint8_t a[2] = { reg >> 8, reg & 0xFF }, d[2] = {0,0};
    struct i2c_msg m[2] = {
        { dev->addr, 0,          2, a },
        { dev->addr, I2C_M_RD,   2, d },
    };
    struct i2c_rdwr_ioctl_data x = { m, 2 };  /* combined transfer -> repeated start */
    if (ioctl(dev->fd, I2C_RDWR, &x) < 0) { perror("cs2600_read_reg"); return -1; }
    *val = ((uint16_t)d[0] << 8) | d[1];
    return 0;
}

int cs2600_modify_reg(struct cs2600 *dev, uint16_t reg, uint16_t mask, uint16_t val) {
    uint16_t cur;
    if (cs2600_read_reg(dev, reg, &cur) < 0) return -1;
    return cs2600_write_reg(dev, reg, (cur & ~mask) | (val & mask));
}

int cs2600_check_id(struct cs2600 *dev) {
    uint16_t id;
    if (cs2600_read_reg(dev, CS2600_REG_DEVICE_ID1, &id) < 0) return -1;
    return id == 0x2600 ? 0 : -1;
}

static int cs2600_unlock(struct cs2600 *dev) {
    if (cs2600_write_reg(dev, CS2600_REG_USER_KEY, 0x00AA) < 0) return -1;
    return cs2600_write_reg(dev, CS2600_REG_USER_KEY, 0x0055);
}

int cs2600_reset(struct cs2600 *dev) {
    if (cs2600_write_reg(dev, CS2600_REG_SW_RESET, 0x005A) < 0) return -1;
    usleep(100000);
    return cs2600_check_id(dev);
}

/* Multiplier Mode, 12MHz REF_CLK_IN/crystal, CLK_IN=48kHz nominal (Pi PWM),
 * MCLK=24.576MHz (ratio 512, datasheet's own worked example, Sec 4.3.2).
 * FLL_BW=1Hz (max jitter rejection - valid since we're on an external ref,
 * not Smart-Multiplier). BCLK=MCLK/8=3.072MHz, FSYNC=MCLK/512=48kHz, I2S.
 * Phase-alignment ENABLED but MANUAL trigger -> passive P_UNLOCK telemetry,
 * chip will NOT silently stretch FSYNC to correct what you're trying to measure. */
int cs2600_init_mult_48k(struct cs2600 *dev) {
    if (cs2600_unlock(dev) < 0) return -1;
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG1, 0x0000);  /* PLL_EN1=0 */
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG2, 0x0000);  /* PLL_EN2=0 */

    cs2600_write_reg(dev, CS2600_REG_PLL_CFG3, 0x0812);
    /* RATIO_CFG=1(12.20), REF_CLK_IN_DIV=÷1 (12MHz->12MHz SYSCLK), SYSCLK_SRC=REF_CLK_IN */

    cs2600_write_reg(dev, CS2600_REG_PLL_CFG4, 0x0000);  /* FLL_BW=1Hz x1 */

    cs2600_write_reg(dev, CS2600_REG_RATIO1_1, 0x2000);  /* ratio 512.0 in Q12.20 */
    cs2600_write_reg(dev, CS2600_REG_RATIO1_2, 0x0000);

    cs2600_write_reg(dev, CS2600_REG_PLL_CFG1, 0x0000);  /* S_RATIO_SEL=0 */
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG2, 0x0001);  /* M_RATIO_SEL=0, MODE=Multiplier */

    cs2600_write_reg(dev, CS2600_REG_OUTPUT_CFG1, 0x5582);
    /* BCLK_DIV=÷8, FSYNC_DIV=÷512, BCLK_INV=1, FSYNC_INV=1 (I2S) */

    cs2600_write_reg(dev, CS2600_REG_OUTPUT_CFG2, 0x8C00);
    /* CLK_IN_INV=1 (matches FSYNC_INV), AUX1_OUT=F_UNLOCK flag, CLK_OUT=MCLK */

    cs2600_write_reg(dev, CS2600_REG_ARC_CFG1, 0x0000);  /* ARC disabled */

    cs2600_write_reg(dev, CS2600_REG_PHASE_ALIGN_CFG1, 0x8045);
    /* EN=1, MODE=Manual, THR=64 MCLK periods (~2.6us) -- tune to your jitter floor */

    cs2600_write_reg(dev, CS2600_REG_UNLOCK_IND, 0x000A); /* clear stale sticky bits */
    cs2600_write_reg(dev, CS2600_REG_ERROR_STS, 0x00FF);

    cs2600_write_reg(dev, CS2600_REG_PLL_CFG1, 0x0100);  /* PLL_EN1=1 */
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG2, 0x0101);  /* PLL_EN2=1 */

    for (int i = 0; i < 50; i++) {                       /* spec: <=~4.2ms @ 48kHz */
        uint16_t u;
        if (cs2600_read_reg(dev, CS2600_REG_UNLOCK_IND, &u) < 0) return -1;
        if (!(u & CS2600_F_UNLOCK_BIT)) return 0;
        usleep(1000);
    }
    return -2;  /* lock timeout */
}

/* Multiplier Mode, 12MHz REF_CLK_IN (on-board crystal Y1), CLK_IN=300Hz
 * (from Pi PWM servo output), CLK_OUT=24.576MHz (ratio 81920 in Q20.12).
 * BCLK=3.072MHz, FSYNC=48kHz, AUX1_OUT=CLK_IN (buffered for i226 SDP).
 * Phase alignment: automatic, threshold 8 MCLK, speed 1 MCLK/FSYNC.
 * Holdover enabled (S_RATIO_SEL != M_RATIO_SEL).
 *
 * Validated on CDB2600-DC-SD via I2C at 0x2F, 2026-07-07. */
int cs2600_init_mult_300hz(struct cs2600 *dev) {
    /* Reset to known state */
    if (cs2600_reset(dev) < 0) return -1;

    /* PLL mode selection (PLL still disabled) */
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG2, 0x0009);
    /* PLL_MODE_SEL=Multiplier, M_RATIO_SEL=Ratio1 */

    /* Multiplication ratio: 81920 in 20.12 fixed-point = 0x1400_0000 */
    cs2600_write_reg(dev, CS2600_REG_RATIO1_1, 0x1400);
    cs2600_write_reg(dev, CS2600_REG_RATIO1_2, 0x0000);

    /* Reference clock: REF_CLK_IN ÷1, SYSCLK_SRC = REF_CLK_IN */
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG3, 0x0012);

    /* FLL bandwidth: 1 Hz (maximum jitter rejection) */
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG4, 0x0000);

    /* Output: BCLK=CLK_OUT/8 (3.072MHz), FSYNC=CLK_OUT/512 (48kHz),
     * BCLK inverted, FSYNC inverted, FSYNC 50% duty */
    cs2600_write_reg(dev, CS2600_REG_OUTPUT_CFG1, 0x5582);

    /* AUX1_OUT = CLK_IN (buffered), CLK_OUT = MCLK */
    cs2600_write_reg(dev, CS2600_REG_OUTPUT_CFG2, 0x0400);

    /* Phase alignment: automatic, threshold=8 MCLK periods */
    cs2600_write_reg(dev, CS2600_REG_PHASE_ALIGN_CFG1, 0x8008);

    /* Enable PLL with holdover (S_RATIO_SEL=Ratio2 != M_RATIO_SEL=Ratio1) */
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG1, 0x0980);
    /* PLL_EN1=1, S_RATIO_SEL=Ratio2 */

    cs2600_write_reg(dev, CS2600_REG_PLL_CFG2, 0x0109);
    /* PLL_EN2=1, PLL_MODE_SEL=Multiplier */

    /* Wait for frequency lock (300Hz -> up to 700 UI = ~2.3s max) */
    for (int i = 0; i < 3000; i++) {
        uint16_t u;
        if (cs2600_read_reg(dev, CS2600_REG_UNLOCK_IND, &u) < 0) return -1;
        if (!(u & CS2600_F_UNLOCK_BIT)) {
            /* Clear sticky bits */
            cs2600_write_reg(dev, CS2600_REG_UNLOCK_IND, 0x000A);
            return 0;
        }
        usleep(1000);
    }
    return -2;  /* lock timeout */
}

int cs2600_get_status(struct cs2600 *dev, cs2600_status_t *st) {
    uint16_t u, e;
    if (cs2600_read_reg(dev, CS2600_REG_UNLOCK_IND, &u) < 0) return -1;
    if (cs2600_read_reg(dev, CS2600_REG_ERROR_STS, &e) < 0) return -1;
    st->freq_locked       = !(u & CS2600_F_UNLOCK_BIT);
    st->phase_locked      = !(u & CS2600_P_UNLOCK_BIT);
    st->freq_unlock_evt   = !!(u & CS2600_F_UNLOCK_STICKY_BIT);
    st->phase_unlock_evt  = !!(u & CS2600_P_UNLOCK_STICKY_BIT);
    st->err_sts = e;
    cs2600_write_reg(dev, CS2600_REG_UNLOCK_IND,
        CS2600_F_UNLOCK_STICKY_BIT | CS2600_P_UNLOCK_STICKY_BIT);  /* clear for next poll */
    if (e) cs2600_write_reg(dev, CS2600_REG_ERROR_STS, e);
    return 0;
}

/* FLL_BW isn't in the "live-writable" tables -> PLL must be disabled around it,
 * else may need a SW_RESET. Output gates cleanly (OUT_GATE_TYPE=F_UNLOCK) during this. */
int cs2600_set_fll_bw(struct cs2600 *dev, uint8_t bw_sel, int mult16) {
    uint16_t c1, c2;
    if (cs2600_read_reg(dev, CS2600_REG_PLL_CFG1, &c1) < 0) return -1;
    if (cs2600_read_reg(dev, CS2600_REG_PLL_CFG2, &c2) < 0) return -1;
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG1, c1 & ~CS2600_PLL_EN1_BIT);
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG2, c2 & ~CS2600_PLL_EN2_BIT);
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG4, ((mult16?1:0)<<7) | ((bw_sel & 7)<<4));
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG1, c1);
    cs2600_write_reg(dev, CS2600_REG_PLL_CFG2, c2);
    return 0;
}

int cs2600_trigger_phase_align(struct cs2600 *dev) {
    return cs2600_modify_reg(dev, CS2600_REG_PHASE_ALIGN_CFG1,
                              CS2600_PHASE_TRIG_BIT, CS2600_PHASE_TRIG_BIT);
}