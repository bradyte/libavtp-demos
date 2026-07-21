#ifndef CS2600_H
#define CS2600_H
#include <stdint.h>

struct cs2600 { int fd; uint8_t addr; };

typedef struct {
    int freq_locked, phase_locked;
    int freq_unlock_evt, phase_unlock_evt;  /* latched since last poll */
    uint16_t err_sts;
} cs2600_status_t;

#define CS2600_REG_PLL_CFG1         0x0002
#define CS2600_REG_PLL_CFG2         0x0004
#define CS2600_REG_RATIO1_1         0x0006
#define CS2600_REG_RATIO1_2         0x0008
#define CS2600_REG_PLL_CFG3         0x0016
#define CS2600_REG_PLL_CFG4         0x001E
#define CS2600_REG_SW_RESET         0x0058
#define CS2600_REG_OUTPUT_CFG1      0x0100
#define CS2600_REG_OUTPUT_CFG2      0x0102
#define CS2600_REG_ARC_CFG1         0x0104
#define CS2600_REG_PHASE_ALIGN_CFG1 0x0108
#define CS2600_REG_DEVICE_ID1       0x0110
#define CS2600_REG_DEVICE_ID2       0x0112
#define CS2600_REG_UNLOCK_IND       0x0114
#define CS2600_REG_ERROR_STS        0x0116
#define CS2600_REG_USER_KEY         0x1104

#define CS2600_I2C_ADDR             0x2F

#define CS2600_PLL_EN1_BIT         (1 << 8)
#define CS2600_PLL_EN2_BIT         (1 << 8)
#define CS2600_F_UNLOCK_BIT        (1 << 0)
#define CS2600_F_UNLOCK_STICKY_BIT (1 << 1)
#define CS2600_P_UNLOCK_BIT        (1 << 2)
#define CS2600_P_UNLOCK_STICKY_BIT (1 << 3)
#define CS2600_PHASE_TRIG_BIT      (1 << 5)

int cs2600_open(const char *i2c_dev_path, uint8_t addr7, struct cs2600 *dev);
void cs2600_close(struct cs2600 *dev);
int cs2600_write_reg(struct cs2600 *dev, uint16_t reg, uint16_t val);
int cs2600_read_reg(struct cs2600 *dev, uint16_t reg, uint16_t *val);
int cs2600_modify_reg(struct cs2600 *dev, uint16_t reg, uint16_t mask, uint16_t val);
int cs2600_check_id(struct cs2600 *dev);
int cs2600_reset(struct cs2600 *dev);
int cs2600_init_mult_48k(struct cs2600 *dev);
int cs2600_init_mult_300hz(struct cs2600 *dev);
int cs2600_get_status(struct cs2600 *dev, cs2600_status_t *st);
int cs2600_set_fll_bw(struct cs2600 *dev, uint8_t bw_sel, int mult16);
int cs2600_trigger_phase_align(struct cs2600 *dev);
#endif